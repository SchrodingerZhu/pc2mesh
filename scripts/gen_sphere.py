import numpy
HEADER = """ply
format ascii 1.0
comment zipper output
element vertex {}
property float x
property float y
property float z
end_header
"""
import numpy as npy
from math import pi, cos, sin
X = []
CNT = 0

for a in npy.arange(-0.5 * pi, 0.5 * pi, 0.01):
    for b in npy.arange(0, 2 * pi, 0.01):
        z = sin(a)
        x = cos(a) * cos(b)
        y = cos(a) * sin(b)
        X.append((x, y, z))
        CNT = CNT + 1


print(HEADER.format(CNT))
for p in X:
    print(p[0], p[1], p[2])





