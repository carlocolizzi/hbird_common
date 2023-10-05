# Hummingbird Navigation Software Stack

**What did you learn from this? What did you not know before this assignment?** 
We learned a lot about using priority queue as data structure in order to implement A* to track the available nodes based off priority. More specifically within the priority queue, getting and storing node information as lists for an open dictionary and closed dictionary.  Along with removing a node from the open list and, based on certain conditions, updating open and closed lists. 


**What was the most difficult aspect of the assignment?**
The most difficult aspect of the assignment was understanding and implementing the right components for an A* algorithm. It took us some time to fully grasp the different methods and functions that could be used along with how to use them efficiently. We also found some challenges figuring out how to cross check neighboring nodes with the lists and make proper adjustments.  


**What was the easiest or most straightforward aspect of the assignment?**
There were a few parts of this assignment that were relatively straight forward. Creating drone restrictions regarding the obstacles that are set is one part, which was done by checking if the obstacle coordinates are less than the coordinates of the current node including the radius of the drone. Setting specific setpoints for the different stages in the agent control was also relatively straight forward.  

**How long did this assignment take? What took the most time (Setup? Figuring out the codebase/ROS2? Coding in Python? Exploring the questions?)?**
This assignment took about ten hours to complete. It took us some time implement the helper functions for node expansion, calculating heuristic, checking for obstacles and managing a set of nodes in the open and closed list in the path planner. It also took time to understand the specific requirements needed for path planning with A* since some of us were new to it. 
 
 
**What did you learn about motion planning (path planning, trajectory planning) that we didn’t explicitly cover in class or in this assignment?**
We learned about the different specific considerations that are to be taken to check if a nodes coordinates are doable with the bounds and obstacles that are already in the environment. We also learned that webots has a different x,y coordinate than the ones already in our system, we had to translate our original waypoints so that they would be accurate in webots. 
 
 
**What more would you like to learn about motion planning?**
Lulu:  I would like to learn more about the different path planning algorithms that could be used along with trajectory planning because we never fully got into it. 
