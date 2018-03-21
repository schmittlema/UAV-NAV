import numpy as np

label_file = open("test_output.txt",'r')
output_file = open("faketest_input.txt",'w')

np.set_printoptions(threshold=np.nan)

r = np.full((100,100),255)
empty = np.full((100,100),0)

for line in label_file:
    l = eval(line)
    new_line = []
    if l[0] == 1:
        new_line.append(r)
        new_line.append(empty)
        new_line.append(empty)
    if l[1] == 1:
        new_line.append(empty)
        new_line.append(r)
        new_line.append(empty)
    if l[2] == 1:
        new_line.append(empty)
        new_line.append(empty)
        new_line.append(r)
    if l[3] == 1:
        new_line.append(empty)
        new_line.append(empty)
        new_line.append(empty)
    if l[4] == 1:
        new_line.append(r)
        new_line.append(r)
        new_line.append(r)

    output_file.write(str(list(np.array(new_line).flatten()))+'\n')
output_file.close()
label_file.close()


