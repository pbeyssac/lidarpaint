#!/usr/local/bin/python3
#
#
# Colorization scripts for the Lidar scans of France distributed by IGN (https://ign.fr/)
#
# Pierre Beyssac -- 2022 -- BSD license
#


import io
import json
import math
import os
import re
import subprocess
import sys
import time
import xml.etree.ElementTree as ET


from PIL import Image
from pyproj import Transformer
import requests


#
# Pattern to extract Lambert93 kilometer coordinates from IGN Lidar file name
#
re_ign_file = re.compile('.*_(\d\d\d\d)_(\d\d\d\d)_LA93')

#
# Random colors for test tiles
#
colors = [(250,0,0),(0,250,0),(0,0,250),(0,250,250),(250,0,250),(250,250,0),(250,250,250),(0,0,0)]

#
# IGN WMTS API documentation:
# https://geoservices.ign.fr/documentation/services/api-et-services-ogc/images-tuilees-wmts-ogc
#
# Lambert93-projected tiles:
# https://geoservices.ign.fr/services-web-experts-lambert-93
#

wmts_GetTile_fmt = 'SERVICE=WMTS&REQUEST=GetTile&VERSION=1.0.0&LAYER=%(layer)s&STYLE=%(style)s&TILEMATRIXSET=%(matrixset_identifier)s&TILEMATRIX=%(zoom)s&TILEROW=%(y)s&TILECOL=%(x)s&FORMAT=%(format)s'
wmts_GetCapabilities_fmt = 'SERVICE=WMTS&VERSION=1.0.0&REQUEST=GetCapabilities'


def osm_marker(p):
  """Return pointer to map on OSM web site."""
  return 'https://www.openstreetmap.org/?mlat=%.5f&mlon=%.5f#map=16/%.5f/%.5f' % (p[0], p[1], p[0], p[1])


class TileCoords(object):
  """Handle tile coordinate conversion."""
  def __init__(self, ortho_ref, target_ref, zoomconfig):
    self.ortho_ref = ortho_ref
    self.target_ref = target_ref
    self.trans_target_to_wmtsref = Transformer.from_crs(self.target_ref, self.ortho_ref)
    self.trans_target_from_wmtsref = Transformer.from_crs(self.ortho_ref, self.target_ref)
    self.trans_wgs84_from_wmtsref = Transformer.from_crs(self.ortho_ref, "WGS84")

    self.x0 = zoomconfig['x0']
    self.y0 = zoomconfig['y0']
    scale_denominator = zoomconfig['scale_denominator']

    # 0.28 millimeter per pixel = hardcoded value from the WMTS standard
    self.mpp = 0.00028*scale_denominator

    self.tile_size_x = zoomconfig['tile_size_x']
    self.tile_size_y = zoomconfig['tile_size_y']
    self.tile_mx = self.tile_size_x*self.mpp
    self.tile_my = self.tile_size_y*self.mpp

  def get_params(self):
    return self.mpp, self.tile_size_x, self.tile_size_y

  def tile_size(self):
    return self.tile_size_x, self.tile_size_y

  def wmts_transform_to_tile(self, lat, lon, transformer):
    """Transform latitude, longitude to x, y tile number at the current zoom level."""
    ax, ay = transformer.transform(lat, lon)
    bx = ax - self.x0
    by = self.y0 - ay
    tile_x = bx/self.tile_mx
    tile_y = by/self.tile_my
    return tile_x, tile_y, ax, ay

  def wmts_tile_to_geo(self, tile_x, tile_y):
    """Convert API tile coordinates at the current zoom level to projection coordinates."""
    bx = tile_x*self.tile_mx
    by = tile_y*self.tile_my
    ax = bx + self.x0
    ay = self.y0 - by
    return ax, ay

  def wmts_transform_from_tile(self, tile_x, tile_y, transformer):
    """Convert API tile coordinates at the current zoom level back to API projection coordinates."""
    ax, ay = self.wmts_tile_to_geo(tile_x, tile_y)
    lat, lon = transformer.transform(ax, ay)
    return lat, lon

  def wmts_target_to_tile(self, lamb_x, lamb_y):
    """Convert target coordinates to tile number at the current zoom level."""
    return self.wmts_transform_to_tile(lamb_x, lamb_y, self.trans_target_to_wmtsref)

  def wmts_target_from_tile(self, tile_x, tile_y):
    """Convert API tile coordinates at the current zoom level to target coordinates."""
    return self.wmts_transform_from_tile(tile_x, tile_y, self.trans_target_from_wmtsref)

  def wmts_wgs84_from_tile(self, tile_x, tile_y):
    """Convert API tile coordinates at the current zoom level to WGS84."""
    return self.wmts_transform_from_tile(tile_x, tile_y, self.trans_wgs84_from_wmtsref)

  def osm_marker(self, tile_x, tile_y):
    """Return pointer to map on OSM web site."""
    p = self.wmts_transform_from_tile(tile_x, tile_y, self.trans_wgs84_from_wmtsref)
    return osm_marker(p)


class LazColorize(object):
  """Colorize IGN .laz files using IGN aerial imagery from their WMTS API.
  """

  def __init__(self, config):
    self.main_config = config
    if not os.path.exists(self.main_config['cache_dir']):
      print("Creating cache directory %s for API tiles" % self.main_config['cache_dir'])
      os.mkdir(self.main_config['cache_dir'])

    # Target is currently set to Lambert93 = EPSG:2154
    self.target_ref = 'EPSG:2154'
    self.target_ref_file = self.target_ref.replace(':', '_')

    self.session = requests.Session()

    self.config_identifier = self.main_config['default_layer']

    # Select by our config id
    self.config = config['layers'][self.config_identifier]
    self.wmts_init(self.config)


  def wmts_init(self, config):
    # WMTS endpoint API key
    self.key = config['key']

    # Layer name to look for in WMTS
    self.layer = config['layer']

    # Zoom level to look for in WMTS
    self.zoom = config['zoom']

    if not self.wmts_load_capabilities():
      print("Unable to load capabilities")
      return

    self.wmts_print_capabilities()

    #
    # Get the WMTS layer configuration, then from that the zoom configuration
    #
    self.layer_config = self.layer_dict[self.layer]
    self.format = self.layer_config['format']
    self.format_ext = self.format.split('/', 1)[1]
    self.matrixset_identifier = self.layer_config['matrixset']

    matrixconfig = self.matrixset_dict[self.layer_config['matrixset']]
    zoomconfig = matrixconfig['zooms'][str(self.zoom)]
    self.ortho_ref = matrixconfig['crs']
    self.ortho_ref_file = matrixconfig['crs'].replace(':', '_')

    self.TC = TileCoords(self.ortho_ref, self.target_ref, zoomconfig)

    self.ni = 0
    self.testmode = False

    mpp, self.tile_size_x, self.tile_size_y = self.TC.get_params()

    print("\nConfigured for key '%s' layer '%s'" % (self.key, self.layer))
    print("Zoom level %s, %.6f meters per pixel, %.2f pixels per kilometer, %dx%d pixels per tile"
      % (self.zoom, mpp, 1000/mpp, self.tile_size_x, self.tile_size_y))


  def wmts_print_capabilities(self):
    print("Available layers on key '%s':" % self.key)
    print()
    for layer in self.layer_dict.values():
      values = layer.copy()
      values['format'] = values['format'].split('/', 1)[1]
      matrixset = self.matrixset_dict[values['matrixset']]
      values['proj'] = matrixset['crs']
      values['zooms'] = ' '.join(matrixset['zooms'].keys())
      print("%(id)-45s\t%(format)-4s\n\tProjection %(proj)-11s zooms %(zooms)s" % values)

  def wmts_load_capabilities(self):
    """Get WMTS capabilities from server."""
    print("Getting WMTS capabilities for config", self.config_identifier, "key", self.key)
    url = self.config['endpoint_url'] % {'key': self.key} + wmts_GetCapabilities_fmt
    r = self.session.get(url)

    if r.status_code != 200:
      return False

    #
    # Now brace yourself for a long bit of XML parsing of the WMTS reply
    #

    xmlns = {
      '': 'http://www.opengis.net/wmts/1.0',
      'gml': 'http://www.opengis.net/gml',
      'ows': 'http://www.opengis.net/ows/1.1',
      'xlink': 'http://www.w3.org/1999/xlink',
      'xsi': 'http://www.w3.org/2001/XMLSchema-instance'
    }

    root = ET.fromstring(r.content)

    #
    # Extract layer list
    #
    layer_dict = {}
    for layer in root.findall('./Contents/Layer', xmlns):
      layer_identifier = layer.find('./ows:Identifier', xmlns).text
      layer_dict[layer_identifier] = {
        'id': layer_identifier,
        'format': layer.find('./Format', xmlns).text,
        'style': layer.find('./Style/ows:Identifier', xmlns).text,
        'matrixset': layer.find('./TileMatrixSetLink/TileMatrixSet', xmlns).text
      }
    self.layer_dict = layer_dict

    #
    # Extract matrix set list
    #
    matrixset_dict = {}

    for tms in root.findall('./Contents/TileMatrixSet', xmlns):
      identifier = tms.find('./ows:Identifier', xmlns).text
      crs = tms.find('./ows:SupportedCRS', xmlns).text
      zoomconfig = {}
      for tm in tms.findall('./TileMatrix', xmlns):
        x0, y0 = [float(coord) for coord in tm.find('./TopLeftCorner', xmlns).text.split(None, 1)]
        zoom_ident = tm.find('./ows:Identifier', xmlns).text
        zoomconfig[zoom_ident] = {
          'scale_denominator': float(tm.find('./ScaleDenominator', xmlns).text),
          'x0': x0, 'y0': y0,
          'tile_size_x': int(tm.find('./TileWidth', xmlns).text),
          'tile_size_y': int(tm.find('./TileHeight', xmlns).text)
        }
      matrixset_dict[identifier] = {'zooms': zoomconfig, 'crs': crs}

    self.matrixset_dict = matrixset_dict
    return True

  def wmts_compute_tile_parameters(self, lambx_km, lamby_km):
    """From a northwest tile origin, compute the integer ranges of tile coordinates
    to get from WMTS.
    Return the ranges + x & y image size + geographical ranges in the WMTS reference system
    """


    txlist = []
    tylist = []

    #
    # Compute the WMTS coordinates of the 4 corners of the Lidar zone.
    #
    # The loop is arranged so that the top left corner is first in the results
    # and the bottom right corner last, in order to get tx1, tx1_orig & al at the end.
    #

    for x in [lambx_km*1000, (lambx_km+1)*1000]:
      for y in [lamby_km*1000, (lamby_km-1)*1000]:
        #
        # Convert to API tile and geographical coordinates
        #
        tx, ty, tx_orig, ty_orig = self.TC.wmts_target_to_tile(x, y)
        txlist.append((tx, tx_orig))
        tylist.append((ty, ty_orig))

    #
    # Get the top left and bottom right corners
    #
    tx1, tx1_orig = txlist[0]
    ty1, ty1_orig = tylist[0]
    tx2, tx2_orig = txlist[-1]
    ty2, ty2_orig = tylist[-1]

    #
    # Compute the bounding box of the WMTS coordinates.
    # This accounts for margins needed for projection with gdalwarp.
    #
    # Round to integer tile coordinates since we can only fetch
    # full tiles.
    #
    txlist.sort()
    tylist.sort()
    txstart = int(txlist[0][0])
    tystart = int(tylist[0][0])
    txend = math.ceil(txlist[-1][0])
    tyend = math.ceil(tylist[-1][0])

    #
    # Convert the rounded bounding box to WMTS geographical coordinates
    # to georeference the image later.
    #
    tx1_orig, ty1_orig = self.TC.wmts_tile_to_geo(txstart, tystart)
    tx2_orig, ty2_orig = self.TC.wmts_tile_to_geo(txend, tyend)

    #
    # Compute the bounds in target coordinates of the final image.
    # Not used except for display, but shows the margins.
    #
    lambert_topleft_x, lambert_topleft_y = self.TC.wmts_target_from_tile(txstart, tystart)
    lambert_bottomright_x, lambert_bottomright_y = self.TC.wmts_target_from_tile(txend, tyend)

    print("API top left %d, %d %s %.2f, %.2f\nTop left on OSM %s"
      % (txstart, tystart, self.target_ref, lambert_topleft_x, lambert_topleft_y,
      self.TC.osm_marker(txstart, tystart)))
    print("API bottom right %d, %d %s %.2f, %.2f\nBottom right on OSM %s"
      % (txend-1, tyend-1, self.target_ref, lambert_bottomright_x, lambert_bottomright_y,
      self.TC.osm_marker(txend, tyend)))

    size_x = (txend-txstart)*self.tile_size_x
    size_y = (tyend-tystart)*self.tile_size_y

    return (txstart, txend, tystart, tyend, size_x, size_y,
            tx1_orig, ty1_orig, tx2_orig, ty2_orig)


  def wmts_fetch_image(self, txstart, txend, tystart, tyend, size_x, size_y, filename):
    """Fetch the given tile coordinate range then save to a file."""
    print("Creating image size %dx%d" % (size_x, size_y))
    full_image = Image.new('RGB', (size_x, size_y), (0,250,0))

    ypix = 0
    for y in range(tystart, tyend):
      xpix = 0
      for x in range(txstart, txend):
        i = self.wmts_fetch_tile(x, y)
        full_image.paste(i, (xpix, ypix))
        xpix += self.tile_size_x
      ypix += self.tile_size_y

    #
    # Write the reassembled (about 1km² + margins to account for reprojection) aerial picture
    #
    full_image.save(filename)

  def get_extent(self, lambx_km, lamby_km, infile):
    self.infile = infile

    (txstart, txend, tystart, tyend, size_x, size_y,
      tx1_orig, ty1_orig, tx2_orig, ty2_orig) = self.wmts_compute_tile_parameters(lambx_km, lamby_km)

    outprefix = "img-%s-%04d-%04d" % (self.zoom, lambx_km, lamby_km)

    image_wmts = outprefix + ".png"
    image_wmts_ortho_georef = "%s.%s.tiff" % (outprefix, self.ortho_ref_file)
    image_wmts_target_georef = "%s.%s.tiff" % (outprefix, self.target_ref_file)

    #
    # Fetch the initial image we are going to work on.
    #
    self.wmts_fetch_image(txstart, txend, tystart, tyend, size_x, size_y, image_wmts)

    #
    # The following could probably be done in fewer steps using
    # GDAL. At least it works.
    #

    #
    # Convert to a georeferenced TIFF
    #
    subprocess.run([
      self.main_config['gdal_translate_path'],
      "-a_nodata", "0",
      "-of", "GTiff",
      "-a_srs", self.ortho_ref,
      "-a_ullr", "%f" % tx1_orig, "%f" % ty1_orig, "%f" % tx2_orig, "%f" % ty2_orig,
      image_wmts,
      image_wmts_ortho_georef])

    if not self.main_config.get('keeptmpfiles', False):
      os.unlink(image_wmts)

    if self.ortho_ref != self.target_ref:
      #
      # The tiles obtained from the API were not in the target coordinates.
      # Project to a new georeferenced TIFF in the target coordinates.
      #
      subprocess.run([
        self.main_config['gdalwarp_path'],
        "-t_srs", self.target_ref,
        "-r", "bilinear",
        image_wmts_ortho_georef, image_wmts_target_georef])

      if not self.main_config.get('keeptmpfiles', False):
        os.unlink(image_wmts_ortho_georef)

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
            "raster": image_wmts_target_georef
        },
        {
            "type": "writers.las",
            "compression": "true",
            "minor_version": "2",
            "dataformat_id": "3",
            "filename": 'color_%04d_%04d_LA93.laz.tmp' % (lambx_km, lamby_km),
            "software_id": "lidarpaint+pdal"
        }
      ]
    }

    with open('pdal_tmp.json', 'w+') as pdalf:
      json.dump(pdal_config, pdalf)

    subprocess.run([self.main_config['pdal_path'], "pipeline", "pdal_tmp.json"])

    os.rename('color_%04d_%04d_LA93.laz.tmp' % (lambx_km, lamby_km), 'color_%04d_%04d_LA93.laz' % (lambx_km, lamby_km))

    if not self.main_config.get('keeptmpfiles', False):
      os.unlink(image_wmts_target_georef)

  def wmts_fetch_tile(self, tile_x, tile_y):
    """Load a tile at current zoom level and coordinates tile_x, tile_y from the cache or API."""
    if self.testmode:
      self.ni += 1
      return Image.new('RGB', (self.tile_size_x, self.tile_size_y), colors[self.ni % 8])

    data = None

    #
    # Temporary compatibility with old cache.
    #
    old_name = os.path.join(config['cache_dir'], 'orig-%s-%d-%d.jpg'
      % (self.zoom, tile_x, tile_y))
    if self.config_identifier == 'PM' and self.layer == 'ORTHOIMAGERY.ORTHOPHOTOS' and os.path.exists(old_name):
      os.rename(old_name,
                os.path.join(config['cache_dir'], '%s-%s-%d-%d.jpeg'
                  % ((self.layer, self.zoom, tile_x, tile_y))))

    orig_name = os.path.join(config['cache_dir'], '%s-%s-%d-%d.%s'
      % (self.layer, self.zoom, tile_x, tile_y, self.format_ext))

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
      url = (self.config['endpoint_url'] % {'key': self.key}
           + wmts_GetTile_fmt % {'zoom': self.zoom, 'x': int(tile_x), 'y': int(tile_y),
              'matrixset_identifier': self.matrixset_identifier,
              'layer': self.layer, 'style': self.layer_config['style'],
              'format': self.format})
      r = self.session.get(url)
      if r.status_code == 200:
        data = r.content
        with open(orig_name, 'wb+') as o:
          o.write(data)
      else:
        print("HTTP error %d on tile %s %s %d, %d" % (r.status_code, self.config_identifier, self.layer, tile_x, tile_y))
        print('URL:', url)

      # Give the API server some slack
      time.sleep(.1)

    if data is not None:
      i = Image.open(io.BytesIO(data))
    else:
      # Tile not found: replace with a red image
      i = Image.new('RGB', (self.tile_size_x, self.tile_size_y), (250,0,0))
    return i


with open('lidarpaint-config.json', 'r') as cf:
  config = json.load(cf)

c = LazColorize(config)

for arg in sys.argv[1:]:
  m = re_ign_file.match(arg)
  if m:
    lambx_km, lamby_km = m.groups()
    print("Processing Lidar tile with Lambert93 kilometer coordinates", lambx_km, lamby_km)
    lambx_km, lamby_km = float(lambx_km), float(lamby_km)
    c.get_extent(lambx_km, lamby_km, arg)
  else:
    print("Cannot extract Lambert93 coordinates from filename %s, skipping" % arg)
