def sendCoordinates(full_path, full_instruction): # (counter, ins)
    options = {"A" : 1, "B" : 2, "C" : 3,
            "D" : 4, "E" : 5, "F" : 6,
            "G" : 7, "H" : 8, "I" : 9,
            "J" : 10, "K" : 11, "L" : 12,
            "M" : 13, "N" : 14, "O" : 15,
            "P" : 16, "Q" : 17, "R" : 18,
            "S" : 19, "T" : 20}
    for i in range(len(full_instruction)): # reset counter meter everytime each path is done
        counter = 0
        instructions = full_instruction[i]
        path = full_path[i]
        for ins in instructions: # call function when sending ins to robot
            if ins[0] == "1":
                tempNum = options[ins[1]]
                print(path[counter : counter+tempNum+1])
                counter = counter + tempNum

full_path = [
    ['1,1,0', '2,1,0', '3,1,0', '4,1,90', '4,2,90', '4,3,90'], 
    ['4,3,180', '3,3,180', '2,3,90', '2,4,90', '2,5,90', '2,6,90', '2,7,90', '2,8,90', '2,9,90', '2,10,90', '2,11,90'], 
    ['2,11,0', '3,11,0', '4,11,0', '5,11,0', '6,11,0', '7,11,90', '7,12,90', '7,13,90', '7,14,90', '7,15,0']
    ]
full_instruction = [
    ['1C', '30', '1C'],
    ['30', '1B', '40', '1I'], 
    ['40', '1E', '30', '1D', '40']
    ]

sendCoordinates(full_path, full_instruction)