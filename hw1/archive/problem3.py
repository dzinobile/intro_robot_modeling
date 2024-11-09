import numpy as np

# Define the image plane distance
z_plane = 5.0

# Construct the transformation matrix H with the correct bottom-right element
H = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [1, 0, 0, 1]
])

A = np.array([[0], [0], [0], [1]])
B = np.array([[1],[0],[0],[1]])
C = np.array([[1],[1],[0],[1]])
D = np.array([[0],[1],[0],[1]])

A1 = np.dot(H,A)
B1 = np.dot(H,B)
C1 = np.dot(H,C)
D1 = np.dot(H,D)
print(A1)
print(B1)
print(C1)
print(D1)