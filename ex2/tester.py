import os 

min_nodes = 4
max_nodes = 9
repeats = 100


for i in range(min_nodes,max_nodes+1):
    vertices = i
    print("N=",vertices)
    for j in range(0,repeats):
        os.system("./generate_random_Kn.tcl " + str(vertices) + " > ./current.dimacs")
        
        os.system("./bin/minimum-mean-cycle ./current.dimacs out.dimacs > /dev/null")

        val_file = open("validation.txt", "r")
        lines = []
        for line in val_file:
            lines.append(line)

        if(lines[0] == lines[1]):
            print(".",end='',flush=True)
        else:
            print("wrong")
            print(lines)
            input()
        val_file.close()
    print("")
    