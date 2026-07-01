#!/usr/bin/env python3
"""Generate farmWith4CropRows.world from farmWith1CropRow.world:
- Remove all trees (oak_tree, pine_tree)
- Expand crop rows from 1 to 4 rows at y = -3, -1, 1, 3
"""
import re
import os

src_path = os.path.join(os.path.dirname(__file__), "farmWith1CropRow.world")
dst_path = os.path.join(os.path.dirname(__file__), "farmWith4CropRows.world")

with open(src_path, 'r') as f:
    content = f.read()

# ============================================================
# Step 1: Remove all tree models (state entries + model defs)
# ============================================================
# Remove tree model entries from everywhere (both state and model definitions)
content = re.sub(
    r"<model name='(?:oak_tree|pine_tree)[^']*'>.*?</model>",
    '',
    content,
    flags=re.DOTALL
)

# Clean up extra blank lines
content = re.sub(r'\n{3,}', '\n\n', content)

# ============================================================
# Step 2: Extract big_plant model definitions from <state> section
# ============================================================
# Find the state section boundaries
state_start = content.find('<state')
state_end = content.find('</state>') + len('</state>')

# Extract big_plant state entries to get x,y positions
big_plant_state_entries = re.findall(
    r"<model name='(big_plant[^']*)'>\s*<pose[^>]*>([\d.-]+)\s+([\d.-]+)\s+([\d.-]+).*?</model>",
    content[state_start:state_end],
    re.DOTALL
)

print(f"Found {len(big_plant_state_entries)} big_plant state entries")

# Extract unique (x, original_name) pairs sorted by x
plant_positions = []
for name, x_str, y_str, z_str in big_plant_state_entries:
    x = float(x_str)
    if not any(abs(p[0] - x) < 0.001 for p in plant_positions):
        plant_positions.append((x, name, float(y_str), float(z_str)))

plant_positions.sort(key=lambda p: p[0])
print(f"Unique x positions: {[p[0] for p in plant_positions]}")

# ============================================================
# Step 3: Remove all big_plant state entries
# ============================================================
before_state = content[:state_start]
state_section = content[state_start:state_end]
after_state = content[state_end:]

# Remove big_plant entries from state section
state_section = re.sub(
    r"<model name='big_plant[^']*'>.*?</model>",
    '',
    state_section,
    flags=re.DOTALL
)

# ============================================================
# Step 4: Add new big_plant state entries for 4 rows
# ============================================================
row_ys = [-3, -1, 1, 3]

new_state_entries = []
for y in row_ys:
    for xi, (x, orig_name, orig_y, orig_z) in enumerate(plant_positions):
        name = orig_name if y == row_ys[0] else f"big_plant_r{row_ys.index(y)+1}_{xi}"
        entry = f"""      <model name='{name}'>
        <pose frame=''>{x} {y} 0 0 -0 0</pose>
        <scale>1 1 1</scale>
        <link name='big_plant_22::link_0'>
          <pose frame=''>{x} {y} 0.111949 0 -0 0</pose>
          <velocity>0 0 0 0 -0 0</velocity>
          <acceleration>0 0 0 0 -0 0</acceleration>
          <wrench>0 0 0 0 -0 0</wrench>
        </link>
      </model>"""
        new_state_entries.append(entry)

# Insert before </state>
state_section = state_section.replace('</state>', '\n'.join(new_state_entries) + '\n</state>')

# ============================================================
# Step 5: Remove all big_plant model definitions from model section
# ============================================================
after_state = re.sub(
    r"<model name='big_plant[^']*'>.*?</model>",
    '',
    after_state,
    flags=re.DOTALL
)

# ============================================================
# Step 6: Generate new big_plant model definitions for 4 rows
# ============================================================
# Get one template model definition from the original file
# Read the original file again for the template
with open(src_path, 'r') as f:
    orig_content = f.read()

# Find the first big_plant model definition (complete one, not from state)
# Look after </state>
orig_state_end = orig_content.find('</state>') + len('</state>')
orig_model_section = orig_content[orig_state_end:]

# Find the first big_plant model definition
first_model_match = re.search(
    r"<model name='big_plant[^']*'>.*?</model>",
    orig_model_section,
    re.DOTALL
)
template = first_model_match.group(0)

print(f"Template model definition length: {len(template)} chars")

# Generate model definitions for each position
new_model_defs = []
for y in row_ys:
    for xi, (x, orig_name, orig_y, orig_z) in enumerate(plant_positions):
        if y == row_ys[0]:
            new_name = orig_name
        else:
            new_name = f"big_plant_r{row_ys.index(y)+1}_{xi}"
        
        # Replace name and pose in template
        # Extract original name from template
        m = re.search(r"<model name='([^']+)'>", template)
        tpl_name = m.group(1)
        
        new_def = template.replace(f"<model name='{tpl_name}'>", f"<model name='{new_name}'>")
        
        # Replace the model pose at the end (the one near </model>)
        # This pose comes after <allow_auto_disable>
        new_def = re.sub(
            r"(<allow_auto_disable>1</allow_auto_disable>\s*<pose[^>]*>)[\d.-]+(\s+)[\d.-]+(\s+)[\d.-]+",
            f"\\g<1>{x}\\g<2>{y}\\g<3>{orig_z}",
            new_def
        )
        
        new_model_defs.append(new_def)

# Insert new model definitions before </world>
after_state = re.sub(
    r'(\s*)</world>',
    '\n' + '\n'.join(new_model_defs) + '\n</world>',
    after_state
)

# ============================================================
# Step 7: Assemble final content
# ============================================================
content = before_state + state_section + after_state

# Clean up extra blank lines
content = re.sub(r'\n{4,}', '\n\n\n', content)

with open(dst_path, 'w') as f:
    f.write(content)

total_plants = len(plant_positions) * len(row_ys)
print(f"\nGenerated {dst_path}")
print(f"Size: {os.path.getsize(dst_path)} bytes")
print(f"Rows: {len(row_ys)} at y = {row_ys}")
print(f"Plants per row: {len(plant_positions)}")
print(f"Total plants: {total_plants}")
print(f"Trees: removed")