import numpy as np
np.set_printoptions(suppress=True, precision=2)

# Camera Matrix
K = np.matrix([[861.7849547322785, 0, 320.0, 0], 
			   [0, 848.6931340450212, 240.0, 0],
			   [0, 0, 1, 0]])
# Distortion
D = np.array([0.1321407125406009, -0.1414210683732252, -0.009010450393461093, 0.06420631003377338, 0])

# uv pixel coords
p1 = np.matrix([[320.0, 240.0, 1]]).T # origin
p2 = np.matrix([[289.0, 240.0, 1]]).T # 10cm left, 2.7 m away
# world X Y Z (Z = depth)
P1 = np.matrix([[0.0, 0.0, 2.7, 1]]).T # origin world
P2 = np.matrix([[-0.1, 0.0, 2.7, 1]]).T # origin world

print K
print "World P1", P1.A1
print "World P2", P2.A1


a = K*P1 
a /= a.A1[2]
print "Guessed p1", a.A1, "vs actual p1", p1.A1

b = K*P2
b /= b.A1[2]
print "Guessed p2", b.A1, "vs actual p2", p2.A1

# p = K*P
# P = K^-1 * p
Kinv = K.I
depth = 2.7 # found using disparity
print Kinv * p1 * depth
print Kinv * p2 * depth