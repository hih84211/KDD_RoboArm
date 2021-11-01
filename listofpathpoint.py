import cnc_input


def A_walkonchip(matrices,gluewidth):
    allsteps = []
    for matrix in matrices:
       allsteps.extend(A_walkinmatrix(matrix[0],matrix[1],matrix[2],int(gluewidth)))                 
    return allsteps

def A_walkinmatrix(matrix,startpoint_x, startpoint_y,gluewidth ):
    walkpoints = []
    for i in range(1, len(matrix),gluewidth):
        for j in range(1,len(matrix[0]),gluewidth):
            walkpoints.append((startpoint_x + i, startpoint_y + j))
    return walkpoints


def main():
    test,gluewidth = cnc_input.main(['-i','right_chip.json'])
    walkpoints = A_walkonchip(test,gluewidth)
    return walkpoints






if __name__ == '__main__':
    main()