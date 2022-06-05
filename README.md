# Implementation of A* on a point robot
This repository contains code to find a path between the start node and the goal node for a point robot using A* algorithm.  

The action set for this project is:  
<img src = "https://github.com/AbhijitMahalle/A_star/blob/master/results/action_set.PNG">  
where, L represents the step size.  

The threshold distance to reach the goal node is 1.5 units without any constraint on the orientation.
## Required libraries:
1. NumPy
2. OpenCV
3. queue
4. math
5. matplotlib  

## Instructions to run the code:
```
python A_star.py
```
Program accepts step size as input. Below output is for step size of 5.
### Node exploration
![](https://github.com/AbhijitMahalle/A_star/blob/master/gif/A_star.gif)  
### Generated path
![](https://github.com/AbhijitMahalle/A_star/blob/master/results/path.png)



