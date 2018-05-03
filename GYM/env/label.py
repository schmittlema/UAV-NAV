#Human Label Code for DAgger Implemetation
import cv2
import numpy as np

infile = "faketest_input.txt"
outfile = "train2_output.txt"
data_path = "/home/ubuntu/Training_data/"

in_file = open(data_path + infile,'r')
lines = in_file.readlines()
out_mat = []

def label(key):
    label = [0,0,0,0,0]
    if key == 119:
        label[2] = 1
    if key == 97:
        label[0] = 1
    if key == 113:
        label[1] = 1
    if key == 100:
        label[4] = 1
    if key == 101:
        label[3] = 1
    return np.array(label)

example = 0
print "Training Examples To Evaluate:",len(lines)
while example < len(lines):
    print "Example:",example
    if example == len(lines) -1:
        print "LAST ONE!"
    image = eval(lines[example])
    resized = np.array(np.reshape(image,(100,100,3)),dtype=np.uint8)
    cv2.imshow("Labeler",resized)
    key = -1
    while key not in [119,97,100,101,113,32,10]:
        key = cv2.waitKey(0)
        print "REDO",key
    while key == 10 and example == len(out_mat):
        key = cv2.waitKey(0)
        print "REDO",key
    while key == 32 and example == 0:
        key = cv2.waitKey(0)
        print "REDO",key
    if key == 32:
        example -=1
    else:
        if key != 10:
            if len(out_mat) == example:
                out_mat.append(label(key))
            else:
                out_mat[example] = label(key)
        example+=1

out_file = open(data_path + outfile,'w')
for line in out_mat:
    out_file.write(str(list(line))+'\n')

in_file.close()
out_file.close()
print "DONE!"
