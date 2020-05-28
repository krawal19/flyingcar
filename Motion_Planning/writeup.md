## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---

## Writeup includes all the rubric points and addressed each one of them.

### Explaining the Starter Code

### 1. What's different about motion_planning.py from the backyard_flyer_solution.py 
```
In motion planning we have two new functions of send_waypoints and plan_path that are not in backyard flyer. The plan_path function creates path using the global position and generated grid. In it a obstacle free path is generated using a_star function and converted to waypoints.
Then send_waypoints is used to send the waypoints to the drone, in which it traverses towards each waypoint.

The main difference is that in backyard flyer there is no planning of the waypoints and there are chances of drone getting collide by the obstacle, where as in motion planning the map is taken into consideration, to avoid it.
```

### 2. Explaining the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
The functionality provided in motion_planning.py that are not in backyward_flyer are as follows:

```
- Argument parser is added for getting arguments --lon, --lat from user.

- MotionPlanning.send_waypoints
Sends waypoints to the simulator.

- MotionPlanning.plan_path
1. Setted the target altitude 10 and saftey distance as 10 
2. Read lon0 (north), lat0 (east) from colliders.csv first line into   
   floating point values.
3. Setting global home position as lon0 and lat0 we got from csv file
4. Retriving current global position and converting to current local 
   position using global_to_local
5. Read obstacle data, create grid using create grid fucnction in 
   planning_utils
6. Setted start value as the current local position subtracted by offset
7. Checks for and goal position in arguments, if yes then uses them as 
   grid_goal else uses default values for goal position
8. Checks if the grid_goal is valid and dosen't on obstacle, if valid then 
   we run astar
9. a_star returns the path to goal which pruned and converted to waypoints 
   using the prune_path function which checks the collinearity of points.
10. Waypoints are sent to send_waypoints
```


The functionality provided in planning_utils.py 
```
- create_grid 
Takes in data from collider.csv file, drone altitude and saftey distance and crates a grid with obstacles marked as 1 and free space as 0, return the the 2d grid.

- action.cost 
Return's the value of the performed action.
- action.delta
Return's the delta movement for the particular action.

- valid_action
Checks if the particular node is off the grid or it's an obstacle
returns list of valid actions that the flying car could take

- a_star
The function is the a_star algorithm, which takes in grid values for obstacles and free space, heuristic function, start and goal.
Then it calculates total cost to gaol from current path and if the action is valid and not previously visited it adds that node into the priority queue. Now once the goal is rached the path is traced back to the start and the cost and path are returned.

- heuristic
Norm of current psoition and the goal, can also be taken euclidean distance.

- point
return set of 3d points

- collinearity_check
checks if there points are collinear if there determinent is less than 
epsilon

- prune_path
Removes the p2 point from the path if  p1,p2,p3 satisfy the linearity condition. Returns the pruned path

- extract_polygons
Creates 3d polygons from the data of the city provided,
Used to check collision of input goal position with the 3d obstacles

- collides
Checks if the given goal point collides with the polygons, returns true if it does else returns false
```

#### 2. Running the script
Start the simulator, select motion planning, If it's inside the box move drone to differnet location using ARMED/DISARMED button and then manual moving it to nearby open location.

For default grid goal
```
python motion_planning.py
```
For custom grid_goal
```
python motion_planning.py --lon -122.392179 --lat 37.790365
```