#!/usr/local/bin/python3
#
#
# Colorization scripts for the Lidar scans of France distributed by IGN (https://ign.fr/)
#
# Pierre Beyssac -- 2022 -- BSD licene
#


import io
import json
import math
import os
import re
import subprocess
import sys
import time


from PIL import Image
from pyproj import Transformer
import requests


# User configuration bits here

cache_dir = '.cache'
pdal_path = '/usr/local/bin/pdal'
gdalwarp_path = '/usr/local/bin/gdalwarp'
gdal_translate_path = '/usr/local/bin/gdal_translate'
keeptmpfiles = False

#
# This can probably be lowered a bit to use lower resolution pictures
#
default_zoom = 19

#
# Pattern to extract Lambert93 kilometer coordinates from IGN Lidar file name
#
re_ign_file = re.compile('.*_\d\d\d\d_(\d\d\d\d)_(\d\d\d\d)_LA93_')

#
# Modify this at your own risk
#
default_key = 'ortho'

#
# Random colors for test tiles
#
colors = [(250,0,0),(0,250,0),(0,0,250),(0,250,250),(250,0,250),(250,250,0),(250,250,250),(0,0,0)]


class LazColorize(object):
  """Colorize IGN .laz files using IGN aerial imagery from their WMTS API.
  IGN WMTS API documentation:
  https://geoservices.ign.fr/documentation/services/api-et-services-ogc/images-tuilees-wmts-ogc
  """

  def __init__(self, zoom=default_zoom, key=default_key):

    if not os.path.exists(cache_dir):
      print("Creating cache directory %s for API tiles" % cache_dir)
      os.mkdir(cache_dir)

    self.ni = 0
    self.testmode = False
    self.identifier = 'PM'
    self.session = requests.Session()

    self.layer = 'ORTHOIMAGERY.ORTHOPHOTOS'
    # Change at your own risk -- untested
    # 'ORTHOIMAGERY.ORTHOPHOTOS.BDORTHO'
    # 'HR.ORTHOIMAGERY.ORTHOPHOTOS'
    # 'ORTHOIMAGERY.ORTHOPHOTOS'

    self.matrixset = self.identifier

    # EPSG:3857 is Web Mercator
    self.ortho_ref = 'EPSG:3857'

    self.trans_lamb93_to_tile = Transformer.from_crs("EPSG:2154", self.ortho_ref)
    self.trans_lamb93_from_tile = Transformer.from_crs(self.ortho_ref, "EPSG:2154")
    self.trans_wgs84_from_tile = Transformer.from_crs(self.ortho_ref, "WGS84")
    self.zoom = zoom
    self.key = key

    #
    # The following parameters should ideally be gotten from
    # https://wxs.ign.fr/ortho/geoportail/wmts?SERVICE=WMTS&VERSION=1.0.0&REQUEST=GetCapabilities
    #
    # I've tried to make it somewhat generic computed from the zoom
    # level instead of using the table from IGN.
    #
    self.x0 = -20037508.3427892476320267
    self.y0 = 20037508.3427892476320267
    scale_denominator = 559082264.0287178958533332/(2**self.zoom)
    self.mpp = 0.00028*scale_denominator

    self.tile_size = 256
    self.tile_m = self.tile_size*self.mpp
    print("Zoom level %d, %.6f meters per pixel, %.2f pixels per kilometer, %d pixels per tile"
      % (self.zoom, self.mpp, 1000/self.mpp, self.tile_size))

  def transform_to_tile(self, lat, lon, transformer):
    """Transform latitude, longitude to x, y tile number at the current zoom level."""
    ax, ay = transformer.transform(lat, lon)
    bx = ax - self.x0
    by = self.y0 - ay
    tile_x = bx/self.tile_m
    tile_y = by/self.tile_m
    return tile_x, tile_y, ax, ay

  def tile_to_geo(self, tile_x, tile_y):
    """Convert API tile coordinates to Web Mercator coordinates at the current zoom level."""
    bx = tile_x*self.tile_m
    by = tile_y*self.tile_m
    ax = bx + self.x0
    ay = self.y0 - by
    return ax, ay

  def transform_from_tile(self, tile_x, tile_y, transformer):
    """Convert API tile coordinates at the current zoom level back to Web Mercator."""
    ax, ay = self.tile_to_geo(tile_x, tile_y)
    lat, lon = transformer.transform(ax, ay)
    return lat, lon

  def lamb93_to_tile(self, lamb_x, lamb_y):
    """Convert Lambert93 coordinates to tile number at the current zoom level."""
    return self.transform_to_tile(lamb_x, lamb_y, self.trans_lamb93_to_tile)

  def lamb93_from_tile(self, tile_x, tile_y):
    """Convert API tile coordinates at the current zoom level to Lambert93."""
    return self.transform_from_tile(tile_x, tile_y, self.trans_lamb93_from_tile)
 
  def wgs84_from_tile(self, tile_x, tile_y):
    """Convert API tile coordinates at the current zoom level to WGS84."""
    return self.transform_from_tile(tile_x, tile_y, self.trans_wgs84_from_tile)

  def osm_marker(self, tile_x, tile_y):
    """Return pointer to map on OSM web site."""
    p = self.transform_from_tile(tile_x, tile_y, self.trans_wgs84_from_tile)
    return 'https://www.openstreetmap.org/?mlat=%.5f&mlon=%.5f#map=16/%.5f/%.5f' % (p[0], p[1], p[0], p[1])

  def get_extent(self, lambx_km, lamby_km, infile):
    self.infile = infile

    #
    # Compute the Lambert93 coordinates of the northwest corner of the Lidar zone.
    # Add margin on every border to account for coordinate conversion with gdalwarp.
    #
    # "70 meters ought to be enough for anybody"
    #

    margin = 70
    lambx1 = lambx_km*1000 - margin
    lamby1 = lamby_km*1000 + margin

    #
    # Compute southeast corner of the Lidar zone + margin
    #
    lambx2 = lambx1 + 1000 + 2*margin
    lamby2 = lamby1 - 1000 - 2*margin

    #
    # Convert to API tile coordinates
    #
    tx1, ty1, tx1_orig, ty1_orig = self.lamb93_to_tile(lambx1, lamby1)
    tx2, ty2, tx2_orig, ty2_orig = self.lamb93_to_tile(lambx2, lamby2)


    # Determine the range for the tiles we are going to get from the API
    txstart, tystart = int(tx1), int(ty1)
    txend, tyend = int(math.ceil(tx2)), int(math.ceil(ty2))

    cropcode = False
    if not cropcode:
      tx1_orig, ty1_orig = self.tile_to_geo(txstart, tystart)
      tx2_orig, ty2_orig = self.tile_to_geo(txend, tyend)

    #
    # Compute the Lambert93 bounds of the final image (not used except for display)
    #
    lambert_topleft_x, lambert_topleft_y = self.lamb93_from_tile(txstart, tystart)
    lambert_bottomright_x, lambert_bottomright_y = self.lamb93_from_tile(txend, tyend)

    print("API top left %d, %d Lambert93 %.2f, %.2f\nTop left on OSM %s"
      % (txstart, tystart, lambert_topleft_x, lambert_topleft_y,
      self.osm_marker(txstart, tystart)))
    print("API bottom right %d, %d Lambert93 %.2f, %.2f\nBottom right on OSM %s"
      % (txend-1, tyend-1, lambert_bottomright_x, lambert_bottomright_y,
      self.osm_marker(txend, tyend)))

    if cropcode:
      size_x = int((tx2-tx1)*self.tile_size + .5)
      size_y = int((ty2-ty1)*self.tile_size + .5)
    else:
      size_x = (txend-txstart)*self.tile_size
      size_y = (tyend-tystart)*self.tile_size
    print("Creating image size %dx%d" % (size_x, size_y))
    full_image = Image.new('RGB', (size_x, size_y), (0,250,0))

    if cropcode:
      crop_left = math.floor((tx1-txstart)*self.tile_size)
      crop_top = math.floor((ty1-tystart)*self.tile_size)
      crop_right = self.tile_size - math.ceil((tx2+txstart)*self.tile_size)
      crop_bottom = self.tile_size - math.ceil((ty2-tystart)*self.tile_size)
      crop_right, crop_bottom = 0, 0

    ypix = 0
    for y in range(tystart, tyend):
      xpix = 0
      for x in range(txstart, txend):
        i = self.fetch_tile(x, y)

        if cropcode:
          left = crop_left if x == txstart else 0
          right = (self.tile_size-crop_right) if x == txend-1 else self.tile_size
          top = crop_top if y == tystart else 0
          bottom = (self.tile_size-crop_bottom) if y == tyend-1 else self.tile_size
          i = i.crop((left,top,right,bottom))
        else:
          left = 0
          right = self.tile_size
          top = 0
          bottom = self.tile_size

        full_image.paste(i, (xpix, ypix))
        xpix += self.tile_size - left
      ypix += self.tile_size - top

    #
    # The following could probably be done in fewer steps using
    # GDAL. At least it works.
    #

    #
    # Write the reassembled (1000 + 2*margin) m² aerial picture as a PNG
    #
    outprefix = "img-%d-%04d-%04d" % (self.zoom, lambx_km, lamby_km)
    full_image.save(outprefix + ".png")

    #
    # Convert to a georeferenced TIFF
    #
    subprocess.run([
      gdal_translate_path,
      "-a_nodata", "0",
      "-of", "GTiff",
      "-a_srs", self.ortho_ref,
      "-a_ullr", "%f" % tx1_orig, "%f" % ty1_orig, "%f" % tx2_orig, "%f" % ty2_orig,
      "%s.png" % outprefix,
      "%s.%s.tiff" % (outprefix, self.ortho_ref)])

    if not keeptmpfiles:
      os.unlink(outprefix+'.png')

    #
    # Project to a new EPSG:2154 (Lambert93) georeferenced TIFF
    #
    subprocess.run([
      gdalwarp_path,
      "-t_srs", "EPSG:2154",
      "%s.%s.tiff" % (outprefix, self.ortho_ref),
      "%s.tiff" % outprefix])

    if not keeptmpfiles:
      os.unlink('%s.%s.tiff' % (outprefix, self.ortho_ref))

    #
    # Run PDAL for the final colorization step using the georeferenced image.
    #

    #
    # Configuration template for PDAL processing
    #
    pdal_config = {
      "pipeline": [
        self.infile,
        {
            "type": "filters.colorization",
            "raster": outprefix+'.tiff'
        },
        {
            "type": "writers.las",
            "compression": "true",
            "minor_version": "2",
            "dataformat_id": "3",
            "filename": 'color-%04d_%04d.laz' % (lambx_km, lamby_km)
        }
      ]
    }

    with open('pdal_tmp.json', 'w+') as pdalf:
      json.dump(pdal_config, pdalf)

    subprocess.run([pdal_path, "pipeline", "pdal_tmp.json"])
    if not keeptmpfiles:
      os.unlink(outprefix+'.tiff')

  def fetch_tile(self, tile_x, tile_y):
    """Load a tile at current zoom level and coordinates tile_x, tile_y from the cache or API."""
    if self.testmode:
      self.ni += 1
      return Image.new('RGB', (self.tile_size, self.tile_size), colors[self.ni % 8])

    data = None

    orig_name = os.path.join(cache_dir, 'orig-%d-%d-%d.jpg' % (self.zoom, tile_x, tile_y))

    #
    # Try the cache first
    #
    if os.path.exists(orig_name):
      with open(orig_name, 'rb') as o:
        data = o.read()
    else:
      #
      # Not found, fetch from the API and store
      #
      url = ('https://wxs.ign.fr/%(key)s/geoportail/wmts?SERVICE=WMTS&REQUEST=GetTile&VERSION=1.0.0&LAYER=%(layer)s&STYLE=normal&TILEMATRIXSET=%(identifier)s&TILEMATRIX=%(zoom)s&TILEROW=%(y)s&TILECOL=%(x)s&FORMAT=image/jpeg' %
         {'zoom': self.zoom, 'x': int(tile_x), 'y': int(tile_y), 'key': self.key, 'identifier': self.identifier, 'layer': self.layer})
      r = self.session.get(url)
      if r.status_code == 200:
        data = r.content
        with open(orig_name, 'wb+') as o:
          o.write(data)

      # Give the API server some slack
      time.sleep(.1)

    if data is not None:
      i = Image.open(io.BytesIO(data))
    else:
      # Tile not found: replace with a red image
      i = Image.new('RGB', (self.tile_size, self.tile_size), (250,0,0))
    return i


c = LazColorize()

for arg in sys.argv[1:]:
  m = re_ign_file.match(arg)
  if m:
    lambx_km, lamby_km = m.groups()
    print("Processing Lidar tile with Lambert93 kilometer coordinates", lambx_km, lamby_km)
    lambx_km, lamby_km = float(lambx_km), float(lamby_km)
    c.get_extent(lambx_km, lamby_km, arg)
  else:
    print("Cannot extract Lambert93 coordinates from filename %s, skipping" % arg)
