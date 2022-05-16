

class StateNode:
    def __init__(self, state, turn):
        self.state = state
        self.turn = turn
        # self.parent = parent
        self.final = self.isFinal() # 0 if draw, 1 if player 1 win, 2 if player 2 win, -1 if game not done
        self.children = []
        self.value = 0

    def isFinal(self):
        lines = [[0,1,2], [3,4,5], [6,7,8], [0,3,6], [1,4,7], [2,5,8], [0,4,8], [2,4,6]]
        for l in lines:
            if self.state[l[0]] == 1 and self.state[l[1]] == 1 and self.state[l[2]] == 1:
                return 1
            if self.state[l[0]] == 2 and self.state[l[1]] == 2 and self.state[l[2]] == 2:
                return 2

        for e in self.state:
            if e == 0: # empty spot so game not over
                return -1
        return 0 # draw

    def getChildren(self):
        print(self.state)
        children = []
        if self.final < 0: # game not over, add children to list
            for i in range(9):
                new_state = [x for x in self.state]
                if (new_state[i] == 0):
                    new_state[i] = self.turn
                    children.append(StateNode(new_state, (1, 2)[self.turn == 1], self))

        self.children = children

    # calculates value of current state
    def staticEval(self):
        if self.final == 1:
            return 1000
        if self.final == 2:
            return -1000

        lines = [[0,1,2], [3,4,5], [6,7,8], [0,3,6], [1,4,7], [2,5,8], [0,4,8], [2,4,6]]
        value = 0
        for l in lines:
            row = [self.state[l[0]], self.state[l[1]], self.state[l[2]]]
            count0 = row.count(0)
            count1 = row.count(1)
            count2 = row.count(2)
            if count1 == 2 and count0 == 1:
                value += 10
            if count2 == 2 and count0 == 1:
                value -= 10
            if count1 == 1 and count0 == 2:
                value += 1
            if count2 == 1 and count0 == 2:
                value -= 1
        return value

class StateTree:
    def __init__(self, state, turn):
        self.root = StateNode(state, 1)

    def minimax(self, state, depth):
        if depth == 0 or state.final>=0:
            state.value = state.staticEval()
            return state.staticEval()

        if state.turn == 1: # maximizing player
            maxEval = -10000
            state.getChildren()
            for child in state.children:
                eval = self.minimax(child, depth-1)
                maxEval = max(maxEval, eval)
            state.value = maxEval
            return maxEval

        if state.turn == 2: # minimizing player
            minEval = 10000
            state.getChildren()
            for child in state.children:
                eval = self.minimax(child, depth-1)
                minEval = min(minEval, eval)
            state.value = minEval
            return minEval


turn0 = StateNode([0,0,0,0,0,0,0,0,0], 1, None)
tree = StateTree()
tree.root.getChildren()
