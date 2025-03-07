import os
import requests
import xml.etree.ElementTree as ET
from PIL import Image
import numpy as np
import cv2

# Constants
API_KEY = os.getenv("GOOGLE_API_KEY")  # Replace with your actual Google Maps API key
ZOOM = 18  # Google Maps zoom level (higher = more detail)
WORLD_SIZE = 1000  # Meters (adjust as needed)
OUTPUT_DIR = "gz_world"
SATELLITE_IMAGE = os.path.join(OUTPUT_DIR, "satellite_texture.png")
SDF_FILE = os.path.join(OUTPUT_DIR, "generated_world.sdf")
ARUCO_TEXTURE = os.path.join(OUTPUT_DIR, "aruco_marker.png")

# Define GPS boundary coordinates (Inter American University of Puerto Rico, Bayamón Campus)
BOUNDARY_COORDS = [
    (18.355304786161437, -66.18271626151996),
    (18.354659666845922, -66.17825060346199),
    (18.348895131624086, -66.18076744699528),
    (18.3490980076, -66.18447478951248)
]

# Compute center latitude and longitude
LAT = sum([coord[0] for coord in BOUNDARY_COORDS]) / len(BOUNDARY_COORDS)
LON = sum([coord[1] for coord in BOUNDARY_COORDS]) / len(BOUNDARY_COORDS)

# Ensure output directory exists
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Fetch satellite image from Google Maps API
MAPS_URL = f"https://maps.googleapis.com/maps/api/staticmap?center={LAT},{LON}&zoom={ZOOM}&size=640x640&maptype=satellite&key={API_KEY}"
response = requests.get(MAPS_URL)
if response.status_code == 200:
    with open(SATELLITE_IMAGE, "wb") as f:
        f.write(response.content)
else:
    raise Exception(f"Failed to download satellite image: {response.status_code} {response.text}")

# Generate a simple black and white ArUco marker
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
marker_img = np.zeros((200, 200), dtype=np.uint8)
marker_id = 0  # Change this for different markers
cv2.aruco.drawMarker(aruco_dict, marker_id, 200, marker_img, 1)
Image.fromarray(marker_img).save(ARUCO_TEXTURE)

# Generate SDF World File
sdf_root = ET.Element("sdf", version="1.9")
world = ET.SubElement(sdf_root, "world", name="generated_world")

# Define spherical coordinates
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
ET.SubElement(script, "uri").text = f"file://{SATELLITE_IMAGE}"

# ArUco marker model
aruco_model = ET.SubElement(world, "model", name="aruco_marker")
ET.SubElement(aruco_model, "static").text = "true"
aruco_link = ET.SubElement(aruco_model, "link", name="aruco_link")
aruco_visual = ET.SubElement(aruco_link, "visual", name="aruco_visual")
aruco_geometry = ET.SubElement(aruco_visual, "geometry")
aruco_plane = ET.SubElement(aruco_geometry, "plane")
ET.SubElement(aruco_plane, "size").text = "0.5 0.5"  # Adjust size as needed
aruco_material = ET.SubElement(aruco_visual, "material")
aruco_script = ET.SubElement(aruco_material, "script")
ET.SubElement(aruco_script, "uri").text = f"file://{ARUCO_TEXTURE}"
ET.SubElement(aruco_model, "pose").text = "0 0 0.01 0 0 0"  # Slightly above the ground

# Save SDF World File
sdf_tree = ET.ElementTree(sdf_root)
sdf_tree.write(SDF_FILE, encoding="utf-8", xml_declaration=True)

print(f"Generated world file: {SDF_FILE}")
print(f"Satellite image saved: {SATELLITE_IMAGE}")
print(f"ArUco marker image saved: {ARUCO_TEXTURE}")
