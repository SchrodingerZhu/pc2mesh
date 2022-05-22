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
edges = [
    [-1.0, -1.0, -1.0],
    [-1.0, -1.0, 1.0],
    [-1.0, 1.0, -1.0],
    [-1.0, 1.0, 1.0],
    [1.0, -1.0, -1.0],
    [1.0, -1.0, 1.0],
    [1.0, 1.0, -1.0],
    [1.0, 1.0, 1.0],
]

origin1 = [-1.0, -1.0, -1.0]
origin2 = [1.0, 1.0, 1.0]

line_target1 = [
    ([1.0, -1.0, -1.0], [-1.0, 1.0, -1.0]),
    ([1.0, -1.0, -1.0], [-1.0, -1.0, 1.0]),
    ([-1.0, 1.0, -1.0], [-1.0, -1.0, 1.0])
]

line_target2 = [
    ([-1.0, 1.0, 1.0], [1.0, -1.0, 1.0]),
    ([-1.0, 1.0, 1.0], [1.0, 1.0, -1.0]),
    ([1.0, -1.0, 1.0], [1.0, 1.0, -1.0])
]



X = []
CNT = 0
for o, targets in [(origin1, line_target1), (origin2, line_target2)]:
    for t in targets:
        p0 = numpy.array(o)

        d0 = numpy.array([
            t[0][0] - o[0],
            t[0][1] - o[1],
            t[0][2] - o[2],
        ])

        d1 = numpy.array([
            t[1][0] - o[0],
            t[1][1] - o[1],
            t[1][2] - o[2],
        ])

        for x in numpy.arange(0, 1, 0.01):
            for y in numpy.arange(0, 1, 0.01):
                p = p0 + x * d0 + y * d1
                X.append(p)
                CNT = CNT + 1

print(HEADER.format(CNT))
for p in X:
    print(p[0], p[1], p[2])





