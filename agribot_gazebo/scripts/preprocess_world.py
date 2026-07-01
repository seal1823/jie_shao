#!/usr/bin/env python3
"""
Pre-process a Gazebo world file to resolve model:// URIs to local file paths.
"""
import os

MODEL_BASE = '/home/zrh/agribot/install/agribot_gazebo/share/agribot_gazebo/models'
SOURCE_WORLD = '/home/zrh/agribot/agribot_gazebo/worlds/farmWith4CropRows.world'
OUTPUT_WORLD = '/home/zrh/agribot/agribot_gazebo/worlds/farm_world_preprocessed.world'

with open(SOURCE_WORLD, 'r') as f:
    content = f.read()

# Replace model:// references with actual paths on disk
content = content.replace('model://mud_box', os.path.join(MODEL_BASE, 'mud_box'))
content = content.replace('model://big_plant', os.path.join(MODEL_BASE, 'big_plant'))

with open(OUTPUT_WORLD, 'w') as f:
    f.write(content)

print(f'Preprocessed world saved to {OUTPUT_WORLD}')