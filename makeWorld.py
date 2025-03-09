import os
import math
import shutil
import requests
from PIL import Image
import xml.etree.ElementTree as ET

# --- Configuration Parameters ---
API_KEY = os.getenv("GOOGLE_API_KEY")  # Google Maps API Key (set this in your environment)
LAT = 18.351306721850296   # Center latitude of the area
LON = -66.18261830442603   # Center longitude of the area
TILE_ZOOM = 20             # Zoom level for satellite tiles (higher = more detail, smaller area per tile)
GRID_SIZE = 12             # Number of tiles to fetch in each dimension (even number -> symmetric around center)
WORLD_SIZE = 2000          # Size of the ground plane in meters (area coverage in Gazebo world)
OUTPUT_DIR = "gz_world"    # Output directory for all generated files
TEXTURES_DIR = os.path.join(OUTPUT_DIR, "materials", "textures")
MODELS_DIR = os.path.join(OUTPUT_DIR, "models")  # Directory for model files (e.g., buildings)

# Ensure API key is provided
if not API_KEY:
    raise RuntimeError("Google API key not found. Please set the GOOGLE_API_KEY environment variable.")

# Start fresh: remove old output and create needed folders
if os.path.exists(OUTPUT_DIR):
    shutil.rmtree(OUTPUT_DIR)
os.makedirs(TEXTURES_DIR, exist_ok=True)
os.makedirs(MODELS_DIR, exist_ok=True)

# --- Google Maps Tile API Session (Satellite imagery) ---
def start_tile_session():
    """Start a session for Google Maps Tiles API (satellite imagery)."""
    session_url = f"https://tile.googleapis.com/v1/createSession?key={API_KEY}"
    payload = {"mapType": "satellite"}  # requesting satellite imagery
    resp = requests.post(session_url, json=payload)
    if resp.status_code == 200:
        session_info = resp.json()
        session_id = session_info.get("session")
        if session_id:
            return session_id
        else:
            raise RuntimeError("Tile API session created but no session ID returned.")
    else:
        raise RuntimeError(f"Failed to create tile session: {resp.status_code} - {resp.text}")

# Initialize session for tile downloads
session_id = start_tile_session()

def lat_lon_to_tile(lat, lon, zoom):
    """Convert latitude and longitude to Google tile X, Y indices at given zoom."""
    # Ensure latitude is clamped within Mercator bounds
    if lat > 85.05112878: lat = 85.05112878
    if lat < -85.05112878: lat = -85.05112878
    n = 2.0 ** zoom
    x = int((lon + 180.0) / 360.0 * n)
    y = int((1.0 - math.log(math.tan(math.radians(lat)) + 1.0 / math.cos(math.radians(lat))) / math.pi) / 2.0 * n)
    return x, y

def download_tile(x, y, zoom, save_path):
    """Download a single map tile (satellite image) using the session token."""
    url = f"https://tile.googleapis.com/v1/2dtiles/{zoom}/{x}/{y}?session={session_id}&key={API_KEY}"
    resp = requests.get(url)
    if resp.status_code == 200:
        with open(save_path, 'wb') as f:
            f.write(resp.content)
    else:
        raise RuntimeError(f"Failed to download tile ({x},{y}): {resp.status_code} - {resp.text}")

# Determine center tile indices for the given lat, lon
center_tx, center_ty = lat_lon_to_tile(LAT, LON, TILE_ZOOM)

# Calculate range of tile indices to download (GRID_SIZE x GRID_SIZE square centered on center tile)
half_grid = GRID_SIZE // 2
tile_paths = []  # to store paths and their relative positions
for j in range(-half_grid, half_grid + 1):
    for i in range(-half_grid, half_grid + 1):
        tx = center_tx + i
        ty = center_ty + j
        tile_filename = f"tile_{TILE_ZOOM}_{tx}_{ty}.jpg"
        tile_path = os.path.join(TEXTURES_DIR, tile_filename)
        download_tile(tx, ty, TILE_ZOOM, tile_path)
        # Store tile image path with offsets for stitching
        tile_paths.append((tile_path, i, j))

# Stitch downloaded tiles into one large image
stitched_texture_path = os.path.join(TEXTURES_DIR, "satellite_texture.jpg")
tile_size = 256  # Google tiles are 256x256 pixels
grid_dim = GRID_SIZE  # number of tiles per side
final_img_size = tile_size * grid_dim
stitched_image = Image.new('RGB', (final_img_size, final_img_size))
for (tile_path, i, j) in tile_paths:
    # Load the tile and paste it at the correct position in the canvas
    img = Image.open(tile_path)
    # Calculate paste position: shift origin to center of grid
    x_off = (i + half_grid) * tile_size
    y_off = (j + half_grid) * tile_size
    stitched_image.paste(img, (x_off, y_off))
# Save the combined satellite texture
stitched_image.save(stitched_texture_path, format='JPEG')

# --- Google Photorealistic 3D Tiles API (3D Buildings) ---
buildings_model_path = None  # will hold the path to the converted building model if available
try:
    # Retrieve the root tileset JSON for photorealistic 3D tiles
    tileset_url = f"https://tile.googleapis.com/v1/3dtiles/root.json?key={API_KEY}"
    resp = requests.get(tileset_url)
    if resp.status_code != 200:
        raise RuntimeError(f"3D Tiles API request failed: {resp.status_code} - {resp.text}")
    tileset = resp.json()
    # The tileset JSON contains a hierarchy of tiles. We need to find tiles covering our area.
    # (This is a simplified approach: in practice, filter by boundingVolume regions.)
    root_tile = tileset.get("root", {})
    building_tile_uri = None
    if root_tile:
        # If the root tile itself has content (unlikely, usually it's subdivided), use it. Otherwise, use a child.
        if "content" in root_tile and "uri" in root_tile["content"]:
            building_tile_uri = root_tile["content"]["uri"]
        elif "children" in root_tile:
            # As a simple heuristic, take the first child tile (for demo purposes or a more refined search can go here)
            for child in root_tile["children"]:
                if "content" in child and "uri" in child["content"]:
                    building_tile_uri = child["content"]["uri"]
                    break
    if not building_tile_uri:
        raise RuntimeError("No building tile found for the specified area (it may not have photorealistic 3D data).")
    # Construct full URL for the tile content (may require session in some cases, here we assume key is enough)
    # Note: building_tile_uri might be relative like "datasets/.../tiles/0/0.b3dm"
    tile_content_url = f"https://tile.googleapis.com{building_tile_uri}?key={API_KEY}"
    tile_resp = requests.get(tile_content_url)
    if tile_resp.status_code != 200:
        raise RuntimeError(f"Failed to download 3D tile content: {tile_resp.status_code} - {tile_resp.text}")
    # Save the downloaded 3D tile data to a file
    tile_data_path = os.path.join(MODELS_DIR, "building_tile.b3dm")
    with open(tile_data_path, "wb") as f:
        f.write(tile_resp.content)
    # Convert the .b3dm tile to a Collada (.dae) or .obj model
    converted_model_name = "buildings.dae"
    converted_model_path = os.path.join(MODELS_DIR, converted_model_name)
    def convert_b3dm_to_collada(b3dm_path, output_path):
        """Placeholder for converting b3dm to Collada/OBJ. Implementation may use external tools or libraries."""
        # In a real scenario, use a library or tool to convert the glTF inside b3dm to DAE/OBJ.
        # For example, one could extract the GLB from b3dm and then use pygltflib or Blender to export to .dae.
        # Here we assume this function fills output_path with the converted model.
        raise NotImplementedError("3D tile conversion not implemented in this script.")
    # Attempt conversion (this will raise NotImplementedError in this placeholder implementation)
    convert_b3dm_to_collada(tile_data_path, converted_model_path)
    buildings_model_path = converted_model_path
except Exception as e:
    print(f"[Warning] 3D buildings not added: {e}")

# --- Generate SDF World XML ---
sdf = ET.Element("sdf", {"version": "1.6"})  # using SDF format version 1.6 for compatibility
world = ET.SubElement(sdf, "world", {"name": "generated_world"})

# Define spherical coordinates (geographic reference for the world)
spherical = ET.SubElement(world, "spherical_coordinates")
ET.SubElement(spherical, "surface_model").text = "EARTH_WGS84"
ET.SubElement(spherical, "latitude_deg").text = str(LAT)
ET.SubElement(spherical, "longitude_deg").text = str(LON)
ET.SubElement(spherical, "elevation").text = "0"
ET.SubElement(spherical, "heading_deg").text = "0"

# Create ground plane model
ground_model = ET.SubElement(world, "model", {"name": "ground_plane"})
ET.SubElement(ground_model, "static").text = "true"
ground_link = ET.SubElement(ground_model, "link", {"name": "ground_link"})
# Visual element for ground (textured plane)
ground_visual = ET.SubElement(ground_link, "visual", {"name": "ground_visual"})
ground_geom = ET.SubElement(ground_visual, "geometry")
ground_plane = ET.SubElement(ground_geom, "plane")
ET.SubElement(ground_plane, "size").text = f"{WORLD_SIZE} {WORLD_SIZE}"
# Apply the stitched satellite texture as the ground texture
ground_material = ET.SubElement(ground_visual, "material")
ET.SubElement(ground_material, "diffuse").text = "1 1 1 1"  # white base (full brightness)
# Use PBR workflow (albedo map) for the texture
pbr = ET.SubElement(ground_material, "pbr")
pbr_metal = ET.SubElement(pbr, "metal")
ET.SubElement(pbr_metal, "albedo_map").text = f"file://{stitched_texture_path}"

# Insert buildings model if available
if buildings_model_path:
    bmodel = ET.SubElement(world, "model", {"name": "buildings"})
    ET.SubElement(bmodel, "static").text = "true"
    blink = ET.SubElement(bmodel, "link", {"name": "buildings_link"})
    ET.SubElement(blink, "pose").text = "0 0 0 0 0 0"  # place at origin
    bvisual = ET.SubElement(blink, "visual", {"name": "buildings_visual"})
    bgeom = ET.SubElement(bvisual, "geometry")
    bmesh = ET.SubElement(bgeom, "mesh")
    ET.SubElement(bmesh, "uri").text = f"file://{buildings_model_path}"
    # (Optional: if the model needs scaling or rotation, add <scale> or adjust pose here)

# Convert the ElementTree to a pretty XML string
sdf_tree = ET.ElementTree(sdf)
xml_str = ET.tostring(sdf, encoding='utf-8')
try:
    import xml.dom.minidom as minidom
    pretty_xml = minidom.parseString(xml_str).toprettyxml(indent="  ")
except Exception:
    pretty_xml = xml_str.decode('utf-8')  # fallback to raw string if pretty print fails

# Save the SDF world file
sdf_file_path = os.path.join(OUTPUT_DIR, "generated_world.sdf")
with open(sdf_file_path, "w", encoding='utf-8') as f:
    f.write(pretty_xml)

print(f"Generated SDF world file: {sdf_file_path}")
print(f"Satellite texture stitched and saved to: {stitched_texture_path}")
if buildings_model_path:
    print(f"Building model added: {buildings_model_path}")
else:
    print("No building model added to the world.")
