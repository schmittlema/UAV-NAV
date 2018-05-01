import numpy as np

label_file = open("test_output.txt",'r')
output_file = open("faketest_input.txt",'w')

np.set_printoptions(threshold=np.nan)

r = np.full((100,100),255)
empty = np.full((100,100),0)
forward = 0.0
left = 0.0
right = 0.0
hard_left = 0.0
hard_right = 0.0

for line in label_file:
    l = eval(line)
    new_line = []
    if l[0] == 1:
        new_line.append(r)
        new_line.append(empty)
        new_line.append(empty)
        hard_left+=1
    if l[1] == 1:
        new_line.append(empty)
        new_line.append(r)
        new_line.append(empty)
        left +=1
    if l[2] == 1:
        new_line.append(empty)
        new_line.append(empty)
        new_line.append(r)
        forward+=1
    if l[3] == 1:
        new_line.append(empty)
        new_line.append(empty)
        new_line.append(empty)
        right +=1
    if l[4] == 1:
        new_line.append(r)
        new_line.append(r)
        new_line.append(r)
        hard_right+=1

    #output_file.write(str(list(np.array(new_line).flatten()))+'\n')
total = float(hard_left + left + forward + right + hard_right)
print hard_left,left,forward,right,hard_right
print hard_left/total,left/total,forward/total,right/total,hard_right/total
output_file.close()
label_file.close()


