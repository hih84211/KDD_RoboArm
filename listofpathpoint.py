import cnc_input
import json
import matplotlib.pyplot as plt


def A_walkonchip(matrices):
    allsteps = []
    for matrix in matrices:
       allsteps.append((A_walkinmatrix(matrix[0],matrix[1],matrix[2]))
    return allsteps

def A_walkinmatrix(matrix,startpoint_x, startpoint_y ):
    walkpoints = []
    for i in range(1, len(matrix),6):
        for j in range(1,len(matrix[0]),6):
            walkpoints.append((startpoint_x + i, startpoint_y + j))
    return walkpoints


def main():
    test = cnc_input.main(['-i','right_chip.json'])
    walkpoints = A_walkonchip(test)
    return walkpoints






if __name__ == '__main__':
    main()