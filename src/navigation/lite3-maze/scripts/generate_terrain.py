#!/usr/bin/env python3
"""Example 4x4 rough maze. Geometry is used only by physics, never by navigation."""
from pathlib import Path
import argparse
import json
import xml.etree.ElementTree as ET
import numpy as np
ROOT = Path(__file__).resolve().parents[1]


def generate(seed=17, maximum_height=.065):
    root = ET.parse(ROOT/'assets/maze_flat.xml').getroot()
    root.set('model', 'lite3_rough_cell_maze')
    world = root.find('worldbody')
    rng = np.random.default_rng(seed)
    cells = []
    for y in range(4):
        for x in range(4):
            severity = rng.uniform(.45, 1.)
            cells.append(dict(cell=[x,y], maximum_height_m=float(maximum_height*severity)))
            for j in range(4):
                for i in range(4):
                    px, py = x+(i+.5)/4, y+(j+.5)/4
                    # Initial stance is clear; every cell has rough surrounding ground.
                    if x == y == 0 and abs(px-.5) < .30 and abs(py-.5) < .30:
                        continue
                    h = rng.uniform(.008, maximum_height*severity)
                    ET.SubElement(world, 'geom', name=f'terrain_{x}_{y}_{i}_{j}', type='box',
                        pos=f'{px} {py} {h/2}', size=f'.12 .12 {h/2}',
                        euler=f'{rng.uniform(-.04,.04)} {rng.uniform(-.04,.04)} 0',
                        rgba=f'{.28+severity*.15} .32 .22 1', group='0', friction='1 .01 .001')
    ET.indent(root)
    ET.ElementTree(root).write(ROOT/'assets/maze_rough.xml', encoding='unicode')
    (ROOT/'assets/terrain_example.json').write_text(json.dumps(dict(seed=seed,
        cell_size_m=1.,cells=cells, note='Example only; official roughness has not been measured.'),indent=2)+'\n')


if __name__ == '__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--seed',type=int,default=17)
    p.add_argument('--maximum-height',type=float,default=.065)
    a=p.parse_args()
    generate(a.seed,a.maximum_height)
