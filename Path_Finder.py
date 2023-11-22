import matplotlib.pyplot as plt
import numpy as np
import random
from matplotlib.pyplot import rcParams
from Obstacle_generator import generate_obstacle
np.set_printoptions(precision=3,suppress=True)
rcParams['font.family']='sans-serif'
rcParams['font.sans-serif']=['Tahoma']
plt.rcParams['font.size']=22

class treeNode:
    def __init__(self,LocationX, LocationY):
        self.LocationX=LocationX
        self.LocationY=LocationY
        self.children=[]
        self.parent=None

class RRTAlgo:
    def __init__(self,start,goal,numIterations,grid,stepSize):
        self.randomTree=treeNode(start[0],start[1])
        self.goal=treeNode(goal[0],goal[1])
        self.nearestNode=None
        self.iterations=min(numIterations,10001)
        self.grid=grid
        self.rho=stepSize
        self.path_distance=0
        self.nearestDist=10000
        self.numWaypoints=0
        self.Waypoints=[]

    def addChild(self,LocationX,LocationY):
        if (LocationX==self.goal.LocationX):
            self.nearestNode.children.append(self.goal)
            self.goal.parent=self.nearestNode
        else:
            tempNode=treeNode(LocationX,LocationY)
            self.nearestNode.children.append(tempNode)
            tempNode.parent=self.nearestNode
    
    def sampleAPoint(self):
        x=random.randint(1,grid.shape[1])
        y=random.randint(1,grid.shape[0])
        point = np.array([x,y])
        return point
    
    def steerToPoint(self, LocationStart, LocationEnd):
        u_hat = self.unitVector(LocationStart, LocationEnd)
        offset = self.rho * u_hat

        new_location = np.array([LocationStart.LocationX + offset[0], LocationStart.LocationY + offset[1]])

        if new_location[0] >= self.grid.shape[1]:
            new_location[0] = self.grid.shape[1]
        if new_location[1] >= self.grid.shape[0]:
            new_location[1] = self.grid.shape[0]

        return new_location
            
    def isInObstacle(self, LocationStart, LocationEnd):
        u_hat = self.unitVector(LocationStart, LocationEnd)
        testPoint = np.array([0.0, 0.0])
        
        for i in range(self.rho):
            testPoint[0] = LocationStart.LocationX + i * u_hat[0]
            testPoint[1] = LocationStart.LocationY + i * u_hat[1]
            
            # Round the indices to the nearest integer
            x_idx = int(round(testPoint[0]))
            y_idx = int(round(testPoint[1]))
            
            
            if 0 <= x_idx < self.grid.shape[1] and 0 <= y_idx < self.grid.shape[0]:
                if np.any(self.grid[y_idx, x_idx] == 1):
                    return True
        
        return False

    
    def unitVector(self,LoactionStart,LocationEnd):
        v=np.array([LocationEnd[0]-LoactionStart.LocationX,LocationEnd[1]-LoactionStart.LocationY])
        u_hat=v/np.linalg.norm(v)
        return u_hat
    
    def findNearest(self,root,point):
        if not root:
            return
        dist = self.distance(root,point)
        if dist<=self.nearestDist:
            self.nearestNode=root
            self.nearestDist=dist
        for child in root.children:
            self.findNearest(child,point)
        pass
            
    
    def distance(self,node1,point):
        dist=np.sqrt((node1.LocationX-point[0])**2+(node1.LocationY-point[1])**2)
        return dist
    
    def goalfound(self,point):
        if self.distance(self.goal,point)<=self.rho:
            return True
        pass  
    
    def resetNearestValues(self):
        self.nearestNode=None
        self.nearestDist=10000
    
    def retraceRRTPath(self,goal):
        if goal.LocationX==self.randomTree.LocationX:
            return
        self.numWaypoints+=1
        currentPoint=np.array([goal.LocationX,goal.LocationY])
        self.Waypoints.insert(0,currentPoint)
        self.path_distance+=self.rho
        self.retraceRRTPath(goal.parent)

generate_obstacle()
plt.close()  

grid = np.load('obstacle_grid.npy')

start = np.array([100.0, 100.0])
goal = np.array([1250.0, 1400.0])

numIterations = 10000
stepSize = 50
startRegion = plt.Circle((start[0], start[1]), stepSize, color='r', fill=True)
goalRegion = plt.Circle((goal[0], goal[1]), stepSize, color='b', fill=False)

fig = plt.figure("RRT Algorithm")
plt.imshow(grid, cmap='binary')
plt.plot(start[0], start[1], 'ro')
plt.plot(goal[0], goal[1], 'bo')
ax = fig.gca()
ax.add_patch(startRegion)
ax.add_patch(goalRegion)


#begin

rrt = RRTAlgo(start,goal,numIterations,grid,stepSize)

for i in range(rrt.iterations):
    rrt.resetNearestValues()
    print("Iteration: ",i)
    
    point=rrt.sampleAPoint()
    rrt.findNearest(rrt.randomTree,point)
    new=rrt.steerToPoint(rrt.nearestNode,point)
    bool=rrt.isInObstacle(rrt.nearestNode,new)
    if(bool==False):
        rrt.addChild(new[0],new[1])
        plt.pause(0.010)
        plt.plot([rrt.nearestNode.LocationX,new[0]],[rrt.nearestNode.LocationY,new[1]],'go',linestyle='--')
        if (rrt.goalfound(new)):
            rrt.addChild(goal[0],goal[1])
            print("Goal Found!!!!")
            break
rrt.retraceRRTPath(rrt.goal)
rrt.Waypoints.insert(0,start)
print("number of waypoiunts: ",rrt.numWaypoints)
print("path distance: ",rrt.path_distance)
print("waypoints: ",rrt.Waypoints)


for i in range(len(rrt.Waypoints)-1):
    plt.plot([rrt.Waypoints[i][0], rrt.Waypoints[i+1][0]],[rrt.Waypoints[i][1],rrt.Waypoints[i+1][1]],'ro',linestyle='--')
    plt.pause(0.010)

plt.show(block=True)