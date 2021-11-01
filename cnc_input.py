import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import json
import sys
import optparse
class cnc_input:
    def __init__(self, photo_name):
        self.photo_name = photo_name
        img = mpimg.imread(self.photo_name)
        gray = img[:,:,2]
        plt.imshow(gray, cmap = plt.get_cmap('gray'))
        self.gray = gray
    
    def target_area(self, x, y, x2, y2):
        target_area = self.gray[ int(y) : int(y2), int(x): int(x2)]
        return target_area
    def target_roi(self, x, y, width, length):
        target_area = self.gray[ int(y) : int(y + length), int(x): int(x + width)]
        return target_area
    def shapes_dealing(item_in_shapes):
        if item_in_shapes['label'] == 'gluewidth':
            gluewidth = item_in_shapes['points'][0] - item_in_shapes['points'][1]
            start_point = -1
            return start_point, gluewidth
        else:
             start_point = item_in_shapes['points'][0]
             end_point = item_in_shapes['points'][2]
             return start_point,end_point
def main(argv = None):
    if argv == None:
        argv = sys.argv
    try:
        target_areas = []
        parser = optparse.OptionParser()
        parser.add_option('-i', '--inputFile', action='store', type='string', dest='input',
                          help='read file from the input path')
        parser.add_option('-o', '--outputFile', action='store', type='string', dest='output',
                          help='output file to the output path')
        parser.add_option('-q', '--quiet', action='store_true', dest='quietMode', help='quiet mode', default=False)
        (options, args) = parser.parse_args(argv)
        
        if options.input:
            try:
                with open(options.input, 'r') as file:
                    data = json.load(file)
                shapes = data['shapes']
                photo = cnc_input('newchip.png')
                for item in shapes :
                    start_point,end_point = cnc_input.shapes_dealing(item)
                    if start_point > 0:
                        target_area = cnc_input.target_area(photo, start_point[0],start_point[1],end_point[0],end_point[1])
                        target_areas.append((target_area,start_point[0],start_point[1]))
                    else:
                        gluewidth = end_point[1]
                return target_areas , gluewidth
            except Exception as e:
                print(e.with_traceback())
    except Exception as e:
        print(e.with_traceback())
'''
please use labelme to produce .json file and draw an area whose lengh is gluewidth,
then named the area 'gluewidth' on its labelname field ,
then better named other areas with something meaningful ,
in order to modify conveniently
'''
        
     


    

    
    
    
if __name__ == '__main__':
    main()
