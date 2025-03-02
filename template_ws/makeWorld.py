import os
import requests
import numpy as np
import xml.etree.ElementTree as ET
import matplotlib.pyplot as plt
from PIL import Image

# Constants
API_KEY = "YOUR_GOOGLE_API_KEY"  # Replace with your actual API key
LAT = 18.351306721850296  # Latitude
LON = -66.18261830442603  # Longitude
GRID_SIZE = 10  # Number of points per side (reduce for testing, increase for higher resolution)
WORLD_SIZE = 1000  # Meters (adjust as needed)
OUTPUT_DIR = "gz_world"

# Ensure output directory exists
os.makedirs(OUTPUT_DIR, exist_ok=True)

# Google Elevation API Request
BASE_URL = "https://maps.googleapis.com/maps/api/elevation/json"
locations = "|".join([f"{LAT + (i - GRID_SIZE//2) * 0.0001},{LON + (j - GRID_SIZE//2) * 0.0001}" 
                        for i in range(GRID_SIZE) for j in range(GRID_SIZE)])
params = {"locations": locations, "key": API_KEY}

try:
    response = requests.get(BASE_URL, params=params, timeout=10)
    response.raise_for_status()
    data = response.json()
    if "results" not in data or len(data["results"]) == 0:
        raise ValueError("No elevation data received. Check API key and quota.")
except requests.exceptions.RequestException as e:
    print(f"Error fetching elevation data: {e}")
    exit(1)

# Process Elevation Data
elevations = np.array([point["elevation"] for point in data["results"]]).reshape((GRID_SIZE, GRID_SIZE))

# Normalize elevations
min_elev, max_elev = elevations.min(), elevations.max()
elevations = (elevations - min_elev) / (max_elev - min_elev) * 255

# Save heightmap image
heightmap_path = os.path.join(OUTPUT_DIR, "heightmap.png")
Image.fromarray(elevations.astype(np.uint8)).save(heightmap_path)

# Generate SDF World File
sdf_root = ET.Element("sdf", version="1.9")
world = ET.SubElement(sdf_root, "world", name="real_location")

# Spherical Coordinates
spherical_coords = ET.SubElement(world, "spherical_coordinates")
ET.SubElement(spherical_coords, "surface_model").text = "EARTH_WGS84"
ET.SubElement(spherical_coords, "latitude_deg").text = str(LAT)
ET.SubElement(spherical_coords, "longitude_deg").text = str(LON)
ET.SubElement(spherical_coords, "elevation").text = str(min_elev)

# Heightmap Model
model = ET.SubElement(world, "model", name="terrain")
ET.SubElement(model, "static").text = "true"
link = ET.SubElement(model, "link", name="terrain_link")

# Collision
collision = ET.SubElement(link, "collision", name="terrain_collision")
geometry = ET.SubElement(collision, "geometry")
heightmap = ET.SubElement(geometry, "heightmap")
ET.SubElement(heightmap, "uri").text = "file://heightmap.png"
ET.SubElement(heightmap, "size").text = f"{WORLD_SIZE} {WORLD_SIZE} {max_elev - min_elev}"
ET.SubElement(heightmap, "pos").text = "0 0 -50"

# Visual
visual = ET.SubElement(link, "visual", name="terrain_visual")
geometry_v = ET.SubElement(visual, "geometry")
heightmap_v = ET.SubElement(geometry_v, "heightmap")
ET.SubElement(heightmap_v, "uri").text = "file://heightmap.png"
ET.SubElement(heightmap_v, "size").text = f"{WORLD_SIZE} {WORLD_SIZE} {max_elev - min_elev}"
ET.SubElement(heightmap_v, "pos").text = "0 0 -50"

# Save SDF World File
sdf_tree = ET.ElementTree(sdf_root)
sdf_path = os.path.join(OUTPUT_DIR, "real_location.sdf")
sdf_tree.write(sdf_path, encoding="utf-8", xml_declaration=True)

print(f"Generated world files saved in: {OUTPUT_DIR}")

# Visualization of the Heightmap
plt.figure()
plt.imshow(elevations, cmap='gray', origin='upper')
plt.colorbar(label='Elevation')
plt.title('Generated Heightmap')
plt.xlabel('X coordinate')
plt.ylabel('Y coordinate')
plt.show()
