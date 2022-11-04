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

# Useful to evaluate the size of WMS images we should fetch
lidar_resolution_m = .2

#
# Pattern to extract Lambert93 kilometer coordinates from IGN Lidar file name
#
re_ign_file = re.compile('.*_(\d\d\d\d)_(\d\d\d\d)_L[AB]93')

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

# WMTS request templates
wmts_GetTile_fmt = 'SERVICE=WMTS&REQUEST=GetTile&VERSION=1.0.0&LAYER=%(layer)s&STYLE=%(style)s&TILEMATRIXSET=%(matrixset_identifier)s&TILEMATRIX=%(zoom)s&TILEROW=%(y)s&TILECOL=%(x)s&FORMAT=%(format)s'
wmts_GetCapabilities_fmt = 'SERVICE=WMTS&VERSION=1.0.0&REQUEST=GetCapabilities'

# WMS request templates
wms_GetCapabilities_fmt = 'SERVICE=WMS&REQUEST=GetCapabilities'
wms_GetMap_fmt = 'LAYERS=%(layer)s&EXCEPTIONS=text/xml&FORMAT=%(format)s&SERVICE=WMS&VERSION=1.3.0&REQUEST=GetMap&STYLES=&CRS=%(crs)s&BBOX=%(bottomleft_lat)f,%(bottomleft_lon)f,%(topright_lat)f,%(topright_lon)f&WIDTH=%(size_x)d&HEIGHT=%(size_y)d'


def osm_marker(p):
  """Return pointer to map on OSM web site."""
  return 'https://www.openstreetmap.org/?mlat=%.5f&mlon=%.5f#map=16/%.5f/%.5f' % (p[0], p[1], p[0], p[1])


def georeference_image(main_config, orig_name, image_ortho_georef, ortho_ref, min_lat, max_lat, min_lon, max_lon):
  """Convert image to a georeferenced TIFF."""

  subprocess.run([
    main_config['gdal_translate_path'],
    "-a_nodata", "0",
    "-of", "GTiff",
    "-a_srs", ortho_ref,
    "-a_ullr", "%f" % min_lon, "%f" % max_lat, "%f" % max_lon, "%f" % min_lat,
    orig_name,
    image_ortho_georef])

  if not main_config.get('keeptmpfiles', False):
    os.unlink(orig_image)


class TileCoords(object):
  """Abstract class for tile coordinate conversion."""
  def get_params(self):
    return self.mpp, self.tile_size_x, self.tile_size_y

  def transform_from_tile(self, tile_x, tile_y, transformer):
    """Convert API tile coordinates at the current zoom level back to API projection coordinates."""
    ax, ay = self.tile_to_geo(tile_x, tile_y)
    lat, lon = transformer.transform(ax, ay)
    return lat, lon

  def target_to_tile(self, lamb_x, lamb_y):
    """Convert target coordinates to tile number at the current zoom level."""
    return self.transform_to_tile(lamb_x, lamb_y, self.trans_target_to_tileref)

  def target_from_tile(self, tile_x, tile_y):
    """Convert API tile coordinates at the current zoom level to target coordinates."""
    return self.transform_from_tile(tile_x, tile_y, self.trans_target_from_tileref)

  def osm_marker(self, tile_x, tile_y):
    """Return pointer to map on OSM web site."""
    p = self.transform_from_tile(tile_x, tile_y, self.trans_wgs84_from_tileref)
    return osm_marker(p)


class WebMercatorTileCoords(TileCoords):
  """WMTS tile coordinate conversion."""
  def __init__(self, ortho_ref, target_ref, zoomconfig, tile_mxy=None):
    self.ortho_ref = ortho_ref
    self.target_ref = target_ref
    self.trans_target_to_tileref = Transformer.from_crs(self.target_ref, self.ortho_ref)
    self.trans_target_from_tileref = Transformer.from_crs(self.ortho_ref, self.target_ref)
    self.trans_wgs84_from_tileref = Transformer.from_crs(self.ortho_ref, "WGS84")

    self.x0 = zoomconfig['x0']
    self.y0 = zoomconfig['y0']
    scale_denominator = zoomconfig['scale_denominator']

    self.tile_size_x = zoomconfig['tile_size_x']
    self.tile_size_y = zoomconfig['tile_size_y']

    if tile_mxy is None:
      # 0.28 millimeter per pixel = hardcoded value from the WMTS standard
      self.mpp = 0.00028*scale_denominator
      self.tile_mx = self.tile_size_x*self.mpp
      self.tile_my = self.tile_size_y*self.mpp
    else:
      self.tile_mx = tile_mxy
      self.tile_my = tile_mxy
      self.mpp = self.tile_mx/self.tile_size_x

  def transform_to_tile(self, lat, lon, transformer):
    """Transform latitude, longitude to x, y tile number at the current zoom level."""
    ax, ay = transformer.transform(lat, lon)
    bx = ax - self.x0
    by = self.y0 - ay
    tile_x = bx/self.tile_mx
    tile_y = by/self.tile_my
    return tile_x, tile_y, ax, ay

  def tile_to_geo(self, tile_x, tile_y):
    """Convert API tile coordinates at the current zoom level to projection coordinates."""
    bx = tile_x*self.tile_mx
    by = tile_y*self.tile_my
    ax = bx + self.x0
    ay = self.y0 - by
    return ax, ay


class TileHandler(object):
  """Common abstract class for tile protocols."""
  def compute_tile_parameters(self, lambx_km, lamby_km):
    """From a northwest tile origin, compute the integer ranges of tile coordinates to get.
    Return the ranges + x & y image size + geographical ranges in the server reference system.
    """
    txlist = []
    tylist = []

    #
    # Compute the coordinates of the 4 corners of the Lidar zone.
    #
    # The loop is arranged so that the top left corner is first in the results
    # and the bottom right corner last, in order to get tx1, tx1_orig & al at the end.
    #

    for x in [lambx_km*1000, (lambx_km+1)*1000]:
      for y in [lamby_km*1000, (lamby_km-1)*1000]:
        #
        # Convert to API tile and geographical coordinates
        #
        tx, ty, tx_orig, ty_orig = self.TC.target_to_tile(x, y)
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
    # Compute the bounding box of the coordinates.
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
    # Convert the rounded bounding box to geographical coordinates
    # to georeference the image later.
    #
    tx1_orig, ty1_orig = self.TC.tile_to_geo(txstart, tystart)
    tx2_orig, ty2_orig = self.TC.tile_to_geo(txend, tyend)

    #
    # Compute the bounds in target coordinates of the final image.
    # Not used except for display, but shows the margins.
    #
    lambert_topleft_x, lambert_topleft_y = self.TC.target_from_tile(txstart, tystart)
    lambert_bottomright_x, lambert_bottomright_y = self.TC.target_from_tile(txend, tyend)

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


  def fetch_image(self, txstart, txend, tystart, tyend, size_x, size_y, filename):
    """Fetch the given tile coordinate range then save to a file."""
    print("Creating image size %dx%d" % (size_x, size_y))
    full_image = Image.new('RGB', (size_x, size_y), (0,250,0))

    ypix = 0
    for y in range(tystart, tyend):
      xpix = 0
      for x in range(txstart, txend):
        i = self.fetch_tile(x, y)
        full_image.paste(i, (xpix, ypix))
        xpix += self.tile_size_x
      ypix += self.tile_size_y

    #
    # Write the reassembled (about 1km² + margins to account for reprojection) aerial picture
    #
    full_image.save(filename)

  def fetch_georeferenced_image(self, lambx_km, lamby_km):
    (txstart, txend, tystart, tyend, size_x, size_y,
      tx1_orig, ty1_orig, tx2_orig, ty2_orig) = self.compute_tile_parameters(lambx_km, lamby_km)

    outprefix = "img-%04d-%04d" % (lambx_km, lamby_km)

    ortho_ref = self.ortho_ref
    ortho_ref_file = ortho_ref.replace(':', '_')
    target_ref_file = self.target_ref.replace(':', '_')

    image = outprefix + ".png"
    image_ortho_georef = "%s.%s.geotiff" % (outprefix, ortho_ref_file)

    #
    # Fetch the initial image we are going to work on.
    #
    self.fetch_image(txstart, txend, tystart, tyend, size_x, size_y, image)

    #
    # Convert to a georeferenced TIFF
    #
    georeference_image(self.main_config, image, image_ortho_georef, ortho_ref,
      ty2_orig, ty1_orig, tx1_orig, tx2_orig)
    return ortho_ref, image_ortho_georef


class WmtsHandler(TileHandler):
  def __init__(self, main_config, config, config_identifier, target_ref):
    self.config = config
    self.main_config = main_config
    self.config_identifier = config_identifier
    self.target_ref = target_ref
    # WMTS endpoint API key
    self.key = config['key']

    # Layer name to look for in WMTS
    self.layer = config['layer']

    # Zoom level to look for in WMTS
    self.zoom = config['zoom']

    self.session = requests.Session()
    if not self.load_capabilities():
      print("Unable to load capabilities")
      return

    self.print_capabilities()

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

    self.TC = WebMercatorTileCoords(self.ortho_ref, target_ref, zoomconfig)

    self.ni = 0
    self.testmode = False

    mpp, self.tile_size_x, self.tile_size_y = self.TC.get_params()

    print("\nConfigured for key '%s' layer '%s', map CRS %s" % (self.key, self.layer, self.ortho_ref))
    print("Zoom level %s, %.6f meters per pixel, %.2f pixels per kilometer, %dx%d pixels per tile"
      % (self.zoom, mpp, 1000/mpp, self.tile_size_x, self.tile_size_y))

  def print_capabilities(self):
    print("Available layers on key '%s':" % self.key)
    print()
    for layer in self.layer_dict.values():
      values = layer.copy()
      values['format'] = values['format'].split('/', 1)[1]
      matrixset = self.matrixset_dict[values['matrixset']]
      values['proj'] = matrixset['crs']
      values['zooms'] = ' '.join(matrixset['zooms'].keys())
      print("%(id)-45s\t%(format)-4s\n\tProjection %(proj)-11s zooms %(zooms)s" % values)

  def load_capabilities(self):
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


  def fetch_tile(self, tile_x, tile_y):
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


# From https://wiki.openstreetmap.org/wiki/TMS
# https://wiki.osgeo.org/wiki/Tile_Map_Service_Specification#Implementation_Advice
#
# Slippy map (OSM)
# https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames#Python

class TmsHandler(TileHandler):
  def __init__(self, main_config, config, config_identifier, target_ref):
    self.config = config
    self.main_config = main_config
    self.config_identifier = config_identifier
    self.target_ref = target_ref

    # Zoom level
    self.zoom = config['zoom']

    self.session = requests.Session()
    self.layer = config_identifier

    # Configure by default for pseudo-Mercator based on WGS84
    self.ortho_ref = self.config.get('crs', 'EPSG:3857')

    # EPSG:3857 is the WebMercator projection based on WGS84.
    #
    # Then the zoomconfig configuration below allows conversion
    # to x, y tile coordinates.
    #
    # This yields the same results as the following formulas
    # from https://wiki.openstreetmap.org/wiki/Slippy_map_tilenames
    # for TMS tile coordinate calculation.
    #
    # def deg2num(self, lat_deg, lon_deg, zoom):
    #   lat_rad = math.radians(lat_deg)
    #   n = 2.0 ** zoom
    #   xtile = (lon_deg + 180.0) / 360.0 * n
    #   ytile = (1.0 - math.asinh(math.tan(lat_rad)) / math.pi) / 2.0 * n
    #   return (xtile, ytile)
    # def num2deg(self, xtile, ytile, zoom):
    #   n = 2.0 ** zoom
    #   lon_deg = xtile / n * 360.0 - 180.0
    #   lat_rad = math.atan(math.sinh(math.pi * (1 - 2 * ytile / n)))
    #   lat_deg = math.degrees(lat_rad)
    #   return (lon_deg, lat_deg)

    zoomconfig = {
      'x0': -6378137*math.pi,
      'y0': 6378137*math.pi,
      'scale_denominator': 1,
      'tile_size_x': 256,
      'tile_size_y': 256
    }

    #
    # Force value for meters per tile for correct coordinate calculation.
    #
    tile_mxy = 6378137*math.pi*2/2**self.zoom

    self.TC = WebMercatorTileCoords(self.ortho_ref, target_ref, zoomconfig, tile_mxy)

    self.ni = 0
    self.testmode = False

    mpp, self.tile_size_x, self.tile_size_y = self.TC.get_params()

    print("\nConfigured for layer '%s'" % (self.layer,))
    print("Zoom level %s, %.6f meters per pixel, %.2f pixels per kilometer, %dx%d pixels per tile"
      % (self.zoom, mpp, 1000/mpp, self.tile_size_x, self.tile_size_y))

    self.switch_n = 0
    self.switch = ['']

    #
    # Parse and convert the URL template.
    #
    tms_re = re.compile('\{([!-]?[a-z0-9]+)(?:\:([a-z0-9,]+))?\}')
    url_template = self.config['endpoint_url']
    while True:
      m = tms_re.search(url_template)
      if not m:
        break
      beg, end = m.span()
      label, arg = m.groups()
      url_template = url_template[:beg] + '%(' + label + ')s' + url_template[end:]
      if label == 'switch':
        self.switch = arg.split(',')
    self.url = url_template

  def fetch_tile(self, tile_x, tile_y):
    """Load a tile at current zoom level and coordinates tile_x, tile_y from the cache or API."""
    if self.testmode:
      self.ni += 1
      return Image.new('RGB', (self.tile_size_x, self.tile_size_y), colors[self.ni % 8])

    data = None

    orig_name = os.path.join(config['cache_dir'], '%s-%s-%d-%d.unk'
      % (self.layer, self.zoom, tile_x, tile_y))

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

      # Respect OSM tile server terms of use for software identification
      headers = {'User-Agent': 'Lidarpaint'}

      # Accepted variables based on the JOSM syntax, see https://josm.openstreetmap.de/wiki/Maps
      url = (self.url
        % {'x': int(tile_x),
           'y': int(tile_y),
           # Yahoo-style Y coordinate
           '!y': int(2**(self.zoom-1)-1-tile_y),
           # OSGeo TMS specification style
           '-y': int(2**self.zoom-1-tile_y),
           'zoom': self.zoom,
           'z': self.zoom,
           'apikey': self.config.get('apikey', 'apikey_not_set'),
           'switch': self.switch[self.switch_n]})
      self.switch_n = (self.switch_n + 1) % len(self.switch)
      r = self.session.get(url, headers=headers)
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

#
# Based on "OpenGIS® Web Map Server Implementation Specification Version: 1.3.0"
# https://portal.ogc.org/files/?artifact_id=14416
#

class WmsHandler(object):
  def __init__(self, main_config, config, config_identifier, target_ref):
    self.config = config
    self.main_config = main_config
    self.config_identifier = config_identifier
    self.target_ref = target_ref

    # WMTS endpoint API key
    self.key = config['key']

    # Layer name to look for in WMS
    self.layer = config['layer']

    self.session = requests.Session()
    if not self.load_capabilities():
      print("Unable to load capabilities")
      return

    self.print_capabilities()

    #
    # Get the WMS layer configuration
    #
    self.layer_config = self.find_layer(self.layer)
    self.format = 'image/geotiff' if 'image/geotiff' in self.formats else self.formats[0]
    self.format_ext = self.format.split('/', 1)[1]

    self.ortho_ref = config['crs']
    self.target_ref = target_ref

    self.ni = 0
    self.testmode = False

    self.trans_target_to_wmsref = Transformer.from_crs(self.target_ref, self.ortho_ref)
    self.trans_target_from_wmsref = Transformer.from_crs(self.ortho_ref, self.target_ref)
    self.trans_wgs84_from_wmsref = Transformer.from_crs(self.ortho_ref, "WGS84")

    min_scale_denominator = self.layer_config['min_scale_denominator']

    # 0.28 millimeter per pixel = hardcoded value from the WMS standard
    mpp = 0.00028*min_scale_denominator
    print("\nConfigured for key '%s' layer '%s'" % (self.key, self.layer))
    if mpp > 0:
      print("Max advertised resolution by server: %.3f meters per pixel, %.2f pixels per kilometer" % (mpp, 1000/mpp))
    else:
      print("Max advertised resolution by server: %.3f meters per pixel" % (mpp,))

    self.size_x = min(1050/lidar_resolution_m, self.maxwidth)
    self.size_y = min(1050/lidar_resolution_m, self.maxheight)
    print("Will use %dx%d" % (self.size_x, self.size_y))


  def load_layer(self, layer, xmlns):
    "Recursively parse XML for a layer."""
    crs_list = []
    for crs in layer.findall('CRS', xmlns):
      crs_list.append(crs.text)
    title = layer.find('Title', xmlns).text
    layer_name = layer.find('Name', xmlns)
    layer_entry = {
      'title': title,
      'crs' : crs_list
    }
    abstract = layer.find('Abstract', xmlns)
    if abstract is not None:
      layer_entry['abstract'] = abstract.text
    style = layer.find('Style')
    if style is not None:
      layer_entry['style'] = layer.find('Style/Name', xmlns).text
    if layer_name is not None:
      layer_name_text = layer_name.text
      layer_entry['name'] = layer_name_text
    minsd = layer.find('MinScaleDenominator', xmlns)
    if minsd is not None:
      layer_entry['min_scale_denominator'] = float(minsd.text)
    maxsd = layer.find('MaxScaleDenominator', xmlns)
    if maxsd is not None:
      layer_entry['max_scale_denominator'] = float(maxsd.text)

    layers = []
    for sub_layer in layer.findall('Layer', xmlns):
      layers.append(self.load_layer(sub_layer, xmlns))
    layer_entry['layer'] = layers
    return layer_entry

  def find_layer(self, name):
    """Find layer by name."""
    layer_entry = {'max_scale_denominator': 1e12, 'min_scale_denominator': 0, 'crs': []}
    return self.find_layer_r(layer_entry, name, self.layers)

  def find_layer_r(self, layer_entry, name, layers):
    """Recursively find layer by name, inheriting attributes."""
    for layer in layers:
      current_entry = layer_entry.copy()
      if 'max_scale_denominator' in layer:
        current_entry['max_scale_denominator'] = layer['max_scale_denominator']
      if 'min_scale_denominator' in layer:
        current_entry['min_scale_denominator'] = layer['min_scale_denominator']
      current_entry['crs'] += layer['crs']
      if layer.get('name', None) == name:
        for attr in ['name', 'title', 'abstract']:
          if attr in layer:
            current_entry[attr] = layer[attr]
        return current_entry
      sub = self.find_layer_r(current_entry, name, layer['layer'])
      if sub:
        return sub
    return None

  def load_capabilities(self):
    print("Getting WMS capabilities for config", self.config_identifier, "key", self.key)
    print()
    url = self.config['endpoint_url'] % {'key': self.key} + wms_GetCapabilities_fmt

    r = self.session.get(url)
    if r.status_code != 200:
      return False
    content = r.content

    xmlns = {
      '': 'http://www.opengis.net/wms',
      'inspire_common': 'http://inspire.ec.europa.eu/schemas/common/1.0',
      'inspire_vs': 'http://inspire.ec.europa.eu/schemas/inspire_vs/1.0',
      'xlink': 'http://www.w3.org/1999/xlink',
      'xsi': 'http://www.w3.org/2001/XMLSchema-instance'
    }

    root = ET.fromstring(content)

    maxwidth = root.find('Service/MaxWidth', xmlns)
    self.maxwidth = int(maxwidth.text) if maxwidth is not None else 40960
    maxheight = root.find('Service/MaxHeight', xmlns)
    self.maxheight = int(maxheight.text) if maxheight is not None else 40960

    formats = []
    layer_dict = {}
    for cap in root.findall('Capability', xmlns):
      for format in cap.findall('Request/GetMap/Format', xmlns):
        formats.append(format.text)
      layers = []
      for layer in cap.findall('Layer', xmlns):
        layers.append(self.load_layer(layer, xmlns))
    self.layers = layers
    self.formats = formats
    return True

  def print_layer(self, layer, indent=0):
    print(indent*' ' + "Title:", layer['title'])
    if 'name' in layer:
      print(indent*' ' + "Name:", layer['name'])
    if 'abstract' in layer:
      print(indent*' ' + "Abstract:", layer['abstract'])
    print()
    for sub_layer in layer['layer']:
      self.print_layer(sub_layer, indent+2)

  def print_capabilities(self):
    print("Available layers on key '%s':" % self.key)
    print()
    for layer in self.layers:
      self.print_layer(layer)

    print("\nAvailable formats:")
    for f in self.formats:
      print('  ' + f.split('/', 1)[1])


  def compute_image_parameters(self, lambx_km, lamby_km):
    """From a northwest tile origin, compute the tile coordinates to get from WMS.
    Return x and y image size + bounding box in the WMS reference system
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
        # Convert to WMS geographical coordinates
        #
        wmsx, wmsy = self.trans_target_to_wmsref.transform(x, y)
        txlist.append(wmsx)
        tylist.append(wmsy)

    #
    # Compute the bounding box of the WMS coordinates.
    # This accounts for margins needed for projection with gdalwarp.
    #
    txlist.sort()
    tylist.sort()
    bottom_lat = txlist[0]
    left_lon = tylist[0]
    top_lat = txlist[-1]
    right_lon = tylist[-1]

    #
    # Compute the bounds in target coordinates of the final image.
    # Not used except for display, but shows the margins.
    #
    lambert_topleft_x, lambert_topleft_y = self.trans_target_from_wmsref.transform(top_lat, left_lon)
    lambert_bottomright_x, lambert_bottomright_y = self.trans_target_from_wmsref.transform(bottom_lat, right_lon)

    print("WMS bounding box: %.6f, %.6f, %.6f, %.6f" % (bottom_lat, left_lon, top_lat, right_lon))
    print("Top left on OSM %s" % osm_marker(self.trans_wgs84_from_wmsref.transform(top_lat, left_lon)))
    print("Bottom right on OSM %s" % osm_marker(self.trans_wgs84_from_wmsref.transform(bottom_lat, right_lon)))

    return bottom_lat, left_lon, top_lat, right_lon, self.size_x, self.size_y


  def fetch_image(self, bottomleft_lat, bottomleft_lon, topright_lat, topright_lon,
                  lambx_km, lamby_km, size_x, size_y, filename):
    """Fetch the given tile coordinate range then save to a file."""
    print("Creating image size %dx%d" % (size_x, size_y))
    if self.testmode:
      self.ni += 1
      return Image.new('RGB', (self.tile_size_x, self.tile_size_y), colors[self.ni % 8])

    data = None

    orig_name = os.path.join(config['cache_dir'], '%s-%04d-%04d.%s'
      % (self.layer, lambx_km, lamby_km, self.format_ext))

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
           + wms_GetMap_fmt % {
              'layer': self.layer, 'size_x': size_x, 'size_y': size_y,
              'bottomleft_lat': bottomleft_lat, 'bottomleft_lon': bottomleft_lon,
              'topright_lat': topright_lat, 'topright_lon': topright_lon,
              'crs': self.ortho_ref,
              'format': self.format})
      r = self.session.get(url, stream=True)
      if r.status_code != 200:
        print("HTTP error %d on tile %s %s %d, %d" % (r.status_code, self.config_identifier, self.layer, lambx_km, lamby_km))
        print('URL:', url)
      else:
        with open(orig_name, 'wb+') as o:
          for data in r.iter_content(1024*1024):
            o.write(data)

      # Give the API server some slack
      time.sleep(.1)

    return orig_name

  def fetch_georeferenced_image(self, lambx_km, lamby_km):
    (bottomleft_lat, bottomleft_lon, topright_lat, topright_lon,
      size_x, size_y) = self.compute_image_parameters(lambx_km, lamby_km)

    outprefix = "img-%04d-%04d" % (lambx_km, lamby_km)

    ortho_ref = self.ortho_ref
    ortho_ref_file = ortho_ref.replace(':', '_')
    target_ref_file = self.target_ref.replace(':', '_')

    image_ortho_georef = "%s.%s.geotiff" % (outprefix, ortho_ref_file)

    #
    # Fetch the initial image we are going to work on.
    #
    orig_name = self.fetch_image(bottomleft_lat, bottomleft_lon, topright_lat, topright_lon,
      lambx_km, lamby_km, size_x, size_y, image_ortho_georef)

    #
    # If necessary, convert to a georeferenced TIFF
    #
    if not orig_name.endswith('.geotiff'):
      georeference_image(self.main_config, orig_name, image_ortho_georef, ortho_ref,
                         bottomleft_lat, topright_lat, bottomleft_lon, topright_lon)
      orig_name = image_ortho_georef
    return ortho_ref, orig_name


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

    self.config_identifier = self.main_config['default_layer']

    # Select by our config id
    self.config = config['layers'][self.config_identifier]

    if self.config['protocol'] == 'wmts':
      self.image_fetcher = WmtsHandler(self.main_config, self.config, self.config_identifier, self.target_ref)
    elif self.config['protocol'] == 'wms':
      self.image_fetcher = WmsHandler(self.main_config, self.config, self.config_identifier, self.target_ref)
    elif self.config['protocol'] == 'tms':
      self.image_fetcher = TmsHandler(self.main_config, self.config, self.config_identifier, self.target_ref)

  def get_extent(self, lambx_km, lamby_km, infile):
    self.infile = infile

    outprefix = "img-%04d-%04d" % (lambx_km, lamby_km)
    ortho_ref, image_ortho_georef = self.image_fetcher.fetch_georeferenced_image(lambx_km, lamby_km)

    if ortho_ref != self.target_ref:
      image_target_georef = "%s.%s.geotiff" % (outprefix, self.target_ref.replace(':', '_'))
      #
      # The tiles obtained from the API were not in the target coordinates.
      # Project to a new georeferenced TIFF in the target coordinates.
      #
      subprocess.run([
        self.main_config['gdalwarp_path'],
        "-t_srs", self.target_ref,
        "-r", "bilinear",
        "-if", "GTiff",
        "-of", "GTiff",
        image_ortho_georef, image_target_georef])
    else:
      image_target_georef = image_ortho_georef

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
            "raster": image_target_georef
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
      os.unlink(image_target_georef)


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
