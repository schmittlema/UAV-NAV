#Human Label Code for DAgger Implemetation
import cv2
import numpy as np

infile = "train3_input.txt"
outfile = "train3_output.txt"
data_path = "/home/ubuntu/Training_data/"

in_file = open(data_path + infile,'r')
lines = in_file.readlines()
example = 2001
out_mat = [0]*example

def label(key):
    label = [0,0,0,0,0]
    if key == 119:
        label[2] = 1
    if key == 97:
        label[4] = 1
    if key == 113:
        label[3] = 1
    if key == 100:
        label[0] = 1
    if key == 101:
        label[1] = 1
    return np.array(label)

example_start = example
print "Training Examples To Evaluate:",len(lines)
while example < len(lines) and example < example_start + 1000:
    print "Example:",example
    if example == len(lines) -1:
        print "LAST ONE!"
    #image = eval(lines[example])
    lin = lines[example][1:-2].split(', ')
    image = map(int,lin)

    resized = np.array(np.reshape(image,(100,100,3)),dtype=np.uint8)
    cv2.imshow("Labeler",resized)
    key = -1
    while key not in [119,97,100,101,113,32,10]:
        key = cv2.waitKey(0)
        print "REDO",key
    while key == 10 and example >= len(out_mat):
        key = cv2.waitKey(0)
        print "REDO",key
    while key == 32 and example == example_start:
        key = cv2.waitKey(0)
        print "REDO",key
    if key == 32:
        example -=1
    else:
        if key != 10:
            if len(out_mat) <= example:
                out_mat.append(label(key))
            else:
                out_mat[example] = label(key)
        example+=1

out_file = open(data_path + outfile,'a')
example = example_start
while example < len(lines) and example < example_start+1000:
    line = out_mat[example]
    out_file.write(str(list(line))+'\n')
    example +=1

in_file.close()
out_file.close()
print "DONE!",example
