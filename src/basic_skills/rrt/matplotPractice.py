#import matplotlib.pyplot as plt
#plt.plot( [1 , 2, 3, 4])
#plt.ylabel(' some numbers ')
#plt.show()

import matplotlib.pyplot as plt
from matplotlib.path import Path
import matplotlib.patches as patches

verts = [
        (0., 0.),  # left, bottom
        (0., 1.),  # left, top
        (-.5, 1.),  # right, top
        (-1., 0.),  # right, bottom
        (0., 0.),  # ignored
        ]

codes = [
        Path.MOVETO,
        Path.LINETO,
        Path.LINETO,
        Path.LINETO,
        Path.LINETO,
        ]

path = Path(verts, codes)

fig, ax = plt.subplots()
patch = patches.PathPatch(path, facecolor='orange', lw=2)

ax.add_patch(patch)
ax.set_xlim(-2, 2)
ax.set_ylim(-2, 2)
plt.show()
