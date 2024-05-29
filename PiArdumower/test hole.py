import numpy as np
import shapely.geometry as sg
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from pathlib import Path
import os
from config import cwd

def add_polygon_patch(coords, ax, fc='blue'):
    patch = patches.Polygon(np.array(coords.xy).T, fc=fc)
    ax.add_patch(patch)
    



fileName = cwd + "/House00/maps01/PERIMETER.npy"
if (os.path.exists(fileName)):
        
        border = np.load(fileName)
        print (border)

        #polygon1 = Polygon(np.squeeze(perimeterArray))
fileName = cwd + "/House00/maps01/EXCLUSION01.npy"
if (os.path.exists(fileName)):
        
        hole = np.load(fileName)
        #print(holes)

#border = [(-10, -10), (-10, 10), (10, 10), (10, -10)]  # Large square
holes = [
    [(10, -30), (15, -30), (15, -20), (10, -18)]
]

region = sg.Polygon(shell=border, holes=holes)

fig, ax = plt.subplots(1, 1)

add_polygon_patch(region.exterior, ax)
for interior in region.interiors:
    add_polygon_patch(interior, ax, 'white')
        
ax.axis('equal')
plt.show()