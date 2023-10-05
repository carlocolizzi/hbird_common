# Hummingbird Navigation Software Stack

## Path Planning

![](https://github.com/carlocolizzi/hbird_common/blob/main/Figure_1.png)
![](https://github.com/carlocolizzi/hbird_common/blob/main/Figure_2.png)
![](https://github.com/carlocolizzi/hbird_common/blob/main/Figure_3.png)
![](https://github.com/carlocolizzi/hbird_common/blob/main/Figure_4.png)

**What did you learn from this? What did you not know before this assignment?** 
We learned a lot about using priority queue as data structure in order to implement A* to track the available nodes based off priority. More specifically within the priority queue, getting and storing node information as lists for an open dictionary and closed dictionary. We also learned about the pros and cons of different types of data structure, which allowed us to pivot from a PriorityQueue to a dictionary of lists.

**What was the most difficult aspect of the assignment?**
The most difficult aspect of the assignment was understanding and implementing the right components for an A* algorithm. It took us some time to fully grasp the different methods and functions that could be used along with how to use them efficiently. We also found some challenges figuring out how to cross check neighboring nodes with the lists and make proper adjustments, as well as debugging the logic of the algorithm

**What was the easiest or most straightforward aspect of the assignment?**
There were a few parts of this assignment that were relatively straight forward. Creating drone restrictions regarding the obstacles that are set is one part, which was done by checking if the obstacle coordinates are less than the coordinates of the current node including the radius of the drone. Setting specific setpoints for the different stages in the agent control was also relatively straight forward.  

**How long did this assignment take? What took the most time (Setup? Figuring out the codebase/ROS2? Coding in Python? Exploring the questions?)?**
This assignment took about 25 hours to complete. It took us some time implement the helper functions for node expansion, calculating heuristic, checking for obstacles and managing a set of nodes in the open and closed list in the path planner. It also took time to understand the specific requirements needed for path planning with A* since we were new to it. Debugging also took a considerable amount of time, given that different pseudocodes found online did not match and that errors are not shown when agent_control_node breaks.
 
**What did you learn about motion planning (path planning, trajectory planning) that we didn’t explicitly cover in class or in this assignment?**
We learned about the different specific considerations that are to be taken to check if a nodes coordinates are doable with the bounds and obstacles that are already in the environment. We also learned that webots has a different x,y coordinate than the ones already in our system, we had to translate our original waypoints so that they would be accurate in webots. We also went more in depth and found the ins and outs of A*.
 
**What more would you like to learn about motion planning?**
Lulu:  I would like to learn more about the different path planning algorithms that could be used along with trajectory planning because we never fully got into it. 
Carlo: I would like to learn more about how kinematics could affect 3D paths and how external factors (wind, etc...) could be taken into account.
