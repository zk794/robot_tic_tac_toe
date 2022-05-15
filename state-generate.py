import numpy as np
import pandas as pd

def generate_states():
    states_0 = [[int(0), int(0), int(0), int(0), int(0), int(0), int(0), int(0), int(0)]]

    states_1 = []
    # one token
    for l in states_0:
        for i in range(9):
            new_state = [x for x in l]
            if (new_state[i] == 0):
                new_state[i] = int(1)
                states_1.append(new_state)

    states_2 = []
    # two tokens
    for l in states_1:
        for i in range(9):
            new_state = [x for x in l]
            if (new_state[i] == 0):
                new_state[i] = int(2)
                states_2.append(new_state)

    states_3 = []
    # three tokens
    for l in states_2:
        for i in range(9):
            new_state = [x for x in l]
            if (new_state[i] == 0):
                new_state[i] = int(1)
                states_3.append(new_state)

    states_4 = []
    # three tokens
    for l in states_3:
        for i in range(9):
            new_state = [x for x in l]
            if (new_state[i] == 0):
                new_state[i] = int(2)
                states_4.append(new_state)

    states_5 = []
    # three tokens
    for l in states_4:
        for i in range(9):
            new_state = [x for x in l]
            if (new_state[i] == 0):
                new_state[i] = int(1)
                states_5.append(new_state)

    states_6 = []
    # three tokens
    for l in states_5:
        for i in range(9):
            new_state = [x for x in l]
            if (new_state[i] == 0):
                new_state[i] = int(2)
                states_6.append(new_state)

    states_7 = []
    # three tokens
    for l in states_6:
        for i in range(9):
            new_state = [x for x in l]
            if (new_state[i] == 0):
                new_state[i] = int(1)
                states_7.append(new_state)

    states_8 = []
    # three tokens
    for l in states_7:
        for i in range(9):
            new_state = [x for x in l]
            if (new_state[i] == 0):
                new_state[i] = int(2)
                states_8.append(new_state)

    states_9 = []
    # three tokens
    for l in states_8:
        for i in range(9):
            new_state = [x for x in l]
            if (new_state[i] == 0):
                new_state[i] = int(1)
                states_9.append(new_state)

    states = states_0 +  states_1 + states_2 + states_3 + states_4 + states_5 + states_6 + states_7 + states_8 + states_9
    df = pd.DataFrame(states)
    with open("states.csv", 'w') as csv_file:
        df.to_csv(path_or_buf = csv_file, index = False, header = False)
    # np.savetxt("states.csv", np.array(states).astype(int))
    # print(states[0])

generate_states()
