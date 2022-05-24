

class StateNode:
    def __init__(self, state, turn, action):
        self.state = state
        self.turn = turn
        self.action = action # which spot is filled to get to this move from the last one
        self.final = self.isFinal() # 0 if draw, 1 if player 1 win, 2 if player 2 win, -1 if game not done
        self.children = []
        self.value = 0

    def isFinal(self):
        lines = [[0,1,2], [3,4,5], [6,7,8], [0,3,6], [1,4,7], [2,5,8], [0,4,8], [2,4,6]]
        for l in lines:
            row = [self.state[l[0]], self.state[l[1]], self.state[l[2]]]
            count0 = row.count(0)
            count1 = row.count(1)
            count2 = row.count(2)
            if count1 == 3:
                return 1
            if count2 == 3:
                return 2

        for e in self.state:
            if e == 0: # empty spot so game not over
                return -1
        return 0 # draw

    def getChildren(self):
        children = []
        if self.final < 0: # game not over, add children to list
            for i in range(9):
                new_state = [x for x in self.state]
                if (new_state[i] == 0):
                    new_state[i] = self.turn
                    children.append(StateNode(new_state, (1, 2)[self.turn == 1], i))

        self.children = children

    # calculates value of current state
    def staticEval(self):
        if self.final == 1:
            return 10000
        if self.final == 2:
            return -10000

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
        self.root = StateNode(state, turn, None)

    # fills values in decision tree
    def minimax(self, node, depth):
        if depth == 0 or node.final>=0:
            node.value = node.staticEval()
            return node.staticEval()

        # if maximizing:
        if node.turn == 1: # maximizing player
            maxEval = -10000
            node.getChildren()
            for child in node.children:
                eval = self.minimax(child, depth-1)
                maxEval = max(maxEval, eval)
            node.value = maxEval
            return maxEval
        # else:
        if node.turn == 2: # minimizing player
            minEval = 10000
            node.getChildren()
            for child in node.children:
                eval = self.minimax(child, depth-1)
                minEval = min(minEval, eval)
            node.value = minEval
            return minEval



    # returns number of the space that the robot should put its chip in
    def pickAction(self, depth):
        self.minimax(self.root, depth)
        next_move = None
        best_val = -10000
        for node in self.root.children:
            if node.value > best_val:
                best_val = node.value
                next_move = node
        return next_move.action
