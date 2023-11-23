# Path-Planning-Mini-Project
This repository contains Python code for creating a path planning environment with randomly generated obstacles using the Shapely and Matplotlib libraries. The repository uses RRT algorithm to navigate through obstacles.

Every time the code is run, it creates 2 files namely 'obstacle_plot.png' and 'obstacle_grid.npy'. obstacle_grid.npy contains obstacels which are randomly formed using random,matplotlib and shapely library



Once you run the code, it will start generating Random nodes and conneting them without intersecting with obstacles. As soon as it reaches the goal it will retrace the path with red color

![](https://github.com/yashmote/Path-Planning-Mini-Project/blob/main/Image_2.png?raw=true)


## Working of the code

there are two python files (Obstacle_generator.py and Path_Finder.py). The Obstacle_generator.py file carries a function that generates 8-25 rectangle using at random spots of plot using shapely and matplotlib library.

Numpy is used to convert plot to .npy file 

Path_Finder.py calls generate_obstacle function from Obstacle_generator.py to generate new obstacles each time when code is run.

it uses RRT algorithm to navigate between start point and goal point 

The plt.plot in Path_Finder.py is for visualisation of running of code. if we remove it the code will execute within fraction of second


