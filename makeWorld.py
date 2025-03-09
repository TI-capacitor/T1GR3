import os
import requests
import numpy as np
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
from PIL import Image
import math

# Constants
API_KEY = os.getenv("GOOGLE_API_KEY")  # Use environment variable for API key
LAT = 18.351306721850296  # Center Latitude
LON = -66.18261830442603  # Center Longitude
WORLD_SIZE = 1000  # Meters (adjust as needed)
OUTPUT_DIR = "gz_world"
SATELLITE_TEXTURE = os.path.join(OUTPUT_DIR, "satellite_texture.png")
SDF_FILE = os.path.join(OUTPUT_DIR, "generated_world.sdf")
TILE_ZOOM = 18  # Adjust as needed

# Ensure output directory exists
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Function to initiate session for Google Maps Tiles API
def start_tile_session():
    session_url = f"https://tile.googleapis.com/v1/createSession?key={API_KEY}"
    session_data = {"mapType": "satellite"}
    response = requests.post(session_url, json=session_data)
    if response.status_code == 200:
        session_info = response.json()
        return session_info.get("session")
    else:
        raise Exception(f"Failed to create tile session: {response.status_code} - {response.text}")

# Start tile session
SESSION_ID = start_tile_session()

# Function to download satellite tile using session
def download_tile(x, y, zoom, output_path):
    tile_url = f"https://tile.googleapis.com/v1/2dtiles/{zoom}/{x}/{y}?session={SESSION_ID}&key={API_KEY}"
    response = requests.get(tile_url)
    if response.status_code == 200:
        with open(output_path, "wb") as f:
            f.write(response.content)
    else:
        raise Exception(f"Failed to download tile ({x}, {y}): {response.status_code} - {response.text}")

# Download a single tile centered on the given location
def lat_lon_to_tile_coords(lat, lon, zoom):
    """Convert latitude and longitude to Google Maps Tile coordinates"""
    n = 2.0 ** zoom
    x = int((lon + 180.0) / 360.0 * n)
    y = int((1.0 - math.log(math.tan(math.radians(lat)) + (1 / math.cos(math.radians(lat)))) / math.pi) / 2.0 * n)
    return x, y

tile_x, tile_y = lat_lon_to_tile_coords(LAT, LON, TILE_ZOOM)
tile_path = os.path.join(OUTPUT_DIR, f"tile_{tile_x}_{tile_y}.png")
download_tile(tile_x, tile_y, TILE_ZOOM, tile_path)

# Generate SDF World File
sdf_root = ET.Element("sdf", version="1.9")
world = ET.SubElement(sdf_root, "world", name="generated_world")

# Define spherical coordinates for Gazebo Harmonic compatibility
spherical_coords = ET.SubElement(world, "spherical_coordinates")
ET.SubElement(spherical_coords, "surface_model").text = "EARTH_WGS84"
ET.SubElement(spherical_coords, "latitude_deg").text = str(LAT)
ET.SubElement(spherical_coords, "longitude_deg").text = str(LON)
ET.SubElement(spherical_coords, "elevation").text = "0"
ET.SubElement(spherical_coords, "heading_deg").text = "0"

# Flat ground plane with satellite texture
model = ET.SubElement(world, "model", name="ground_plane")
ET.SubElement(model, "static").text = "true"
link = ET.SubElement(model, "link", name="ground_link")
visual = ET.SubElement(link, "visual", name="ground_visual")
geometry = ET.SubElement(visual, "geometry")
plane = ET.SubElement(geometry, "plane")
ET.SubElement(plane, "size").text = f"{WORLD_SIZE} {WORLD_SIZE}"
material = ET.SubElement(visual, "material")
script = ET.SubElement(material, "script")
ET.SubElement(script, "uri").text = f"file://{tile_path}"

# Placeholder for 3D Buildings
buildings_model = ET.SubElement(world, "model", name="buildings")
ET.SubElement(buildings_model, "static").text = "true"
buildings_link = ET.SubElement(buildings_model, "link", name="buildings_link")
ET.SubElement(buildings_link, "pose").text = "0 0 0 0 0 0"

# Save SDF World File
sdf_tree = ET.ElementTree(sdf_root)
sdf_tree.write(SDF_FILE, encoding="utf-8", xml_declaration=True)

print(f"Generated world file: {SDF_FILE}")
print(f"Satellite texture saved: {tile_path}")

# Visualization of the Satellite Texture
stitched_image = Image.open(tile_path)
stitched_image.show()
