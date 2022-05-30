# Tic-Tac-Toe Bots

## Project Description: Describe the goal of your project, why it's interesting, what you were able to make your robot do, and what the main components of your project are and how they fit together - please include diagrams and gifs when appropriate

Our goal was to code an intelligent robot opponent for a person to play tic-tac-toe against. This was an interesting project because we had to first think about how we could design a board that was usable for both human and robot players. We were able to make our robot wait until the person makes a move, examine the state of the board and make a move based on what it sees, and pick up and place its pieces.

The main components of our robot are the minimax algorithm, perception, and movement. The algorithm lets the robot decide what to do, then to execute the specific action the robot's perception informs where it will move. Basically, the robot works in a loop:
1. (perception) wait for human move,
2. (algorithm) decide on next move based on where the human moved,
3. (movement & perception) while correct colored tube not in sight, rotate, then move to tube
4. (movement) pick up tube
5. (movement & perception) rotate until home base is in sight
6. (movement & perception) move to home base and spin to face board
7. (movement & perception) move to correct column
8. (movement) place piece at correct height
9. (movement & perception) move to home base and spin to face board
[back to 1]

## System Architecture: Describe in detail the robotics algorithm you implemented and each major component of your project, highlight what pieces of code contribute to these main components

The new algorithm we explored is the minimax algorithm. It is a useful algorithm for two-player games. The main logic for it is in states_tree.py. The idea behind minimax is that you look at the current state of the game, then create a decision tree for a certain number of moves ahead (we chose a depth of 3). So the root would be the current state, then each child would be each of the possibilities for what the board would look like after the player places a new piece, and the same for each of those children. So an empty board has 9 children, a board with one piece has 8 children, etc. If a game is either won or a draw, it has no children. Once the decision tree is made, a score must be assigned to each game state. The score is maximum if the robot wins, minimum if the human wins. We fill in the in between scores by giving points based on how many incomplete lines there are, for example a line of two robot pieces and one empty square is worth more than a line of one robot piece and two empty squares, and a line of one robot and one human piece is worth nothing. Finally, once the values are assigned to the decision tree's nodes, we recursively decide which move will lead to the highest score outcome down the line, assuming that the opposing player will choose moves that lead to the lowest scoring game state (closer to a win for them). In states_tree.py, there is a StateNode class to define one node on the decision tree and evaluate the score of a given game state, and a StateTree class to create the decision tree and run the minimax algorithm.

## Execution: Describe how to run your code, e.g., step-by-step instructions on what commands to run in each terminal window to execute your project code.

Terminals:
1. roscore
2. ssh into robot, set_ip and bringup
3. bringup_cam
4. rosrun image_transport republish compressed in:=raspicam_node/image raw out:=camera/rgb/image_raw
5. roslaunch turtlebot3_manipulation_bringup turtlebot3_manipulation_bringup.launch
6. roslaunch turtlebot3_manipulation_moveit_config move_group.launch
7. rosrun robot_tic_tac_toe execute.py

## Challenges

- Board design: we realized that having all the robot’s pieces being tubes of equal length would not work because lower placed tubes would get in the way of trying to place higher ones. We solved this by redesigning the pieces and the robot's  actions such that the robot will put shorter pieces in the bottom row and longer pieces in the top row.
- Perception: we had all nine AR tags visible so the robot could keep track of open spaces, but the robot could not actually register all of them at once. We overcame this by changing strategy and revealing  only one tag to the robot at once.


## Future Work

- The robot makes some extra unnecessary movements, such as extra spins. We could make it play faster by streamlining its movement.
- Try having the robot draw Xs for its move on a whiteboard using similar strategies as the Robot Forger group; there might be fewer moving parts with that strategy than trying to pick up and stick pieces to the board like we did.
- We could experiment with different algorithms.

## Takeaways:

- We were able to split the work pretty effectively while still helping each other and communicating.
- We found it important to be flexible in our plans; if we had stuck with our original design instead of modifying the board as we went, it would have made it far more difficult for us to have a working product. We had to consider both the robot's and our own limitatations.
