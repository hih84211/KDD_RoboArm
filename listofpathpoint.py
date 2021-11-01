import cnc_input
import json
import matplotlib.pyplot as plt


def A_walkonchip(matrices):
    allsteps = []
    for matrix in matrices:
       allsteps.d(A_walkinmatrix(matrix[0],matrix[1],matrix[2]))
    return allsteps

def A_walkinmatrix(matrix,startpoint_x, startpoint_y ):
    walkpoints = []
    for i in range(1, len(matrix),6):
        for j in range(1,len(matrix[0]),6):
            walkpoints.append((startpoint_x + i, startpoint_y + j))
    return walkpoints

test = cnc_input.main(['-i','right_chip.json'])
print(test[6][0])
plt.imshow(test[6][0], cmap = plt.get_cmap('gray'))
A_walkonchip(test)
