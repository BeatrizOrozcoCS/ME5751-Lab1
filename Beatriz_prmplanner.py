import cv2
import numpy as np
import math
import random
import time
from PIL import Image, ImageTk

from Path import *
# from Queue import Queue

class prm_node:
    def __init__(self,map_i=int(0),map_j=int(0)):
        self.map_i = map_i
        self.map_j = map_j
        self.edges = [] #edges of child nodes
        self.parent = None #parent node


class prm_edge:
    def __init__(self,node1=None,node2=None):
        self.node1 = node1 #parent node
        self.node2 = node2 #child node

#You may modify it to increase efficiency as list.append is slow
class prm_tree:
    def __init__(self):
        self.nodes = []
        self.edges = []

    def add_nodes(self,node):
        self.nodes.append(node)

    #add an edge to our PRM tree, node1 is parent, node2 is the kid
    def add_edges(self,node1,node2): 
        edge = prm_edge(node1,node2)
        self.edges.append(edge)
        node1.edges.append(edge)
        node2.parent=edge.node1
    
    def access_nodes(self,i):
        node_pos = [self.nodes[i].map_i, self.nodes[i].map_j]
        return node_pos


class path_planner:
    def __init__(self,graphics):
        self.graphics = graphics
        # self.graphics.scale = 400 #half pixel number on canvas, the map should be 800 x 800
        # self.graphics.environment.robots[0].set_bot_size(body_cm = 2*self.inflation_radius)
        #self.graphics.environment.width/height = 2

        self.costmap = self.graphics.map
        self.map_width = self.costmap.map_width
        self.map_height = self.costmap.map_height

        self.pTree=prm_tree()

        self._init_path_img()
        self.path = Path()
        
        self.set_start(world_x = .0, world_y = .0)
        self.set_goal(world_x = 0.0, world_y = 0.0, world_theta = .0)

        #self.plan_path()
        #self._show_path()

    def set_start(self, world_x = 0, world_y = 0, world_theta = 0):
        self.start_state_map = Pose()
        map_i, map_j = self.world2map(world_x,world_y)
        print("Start with %d, %d on map"%(map_i,map_j))
        self.start_state_map.set_pose(map_i,map_j,world_theta)
        self.start_node = prm_node(map_i,map_j)
        self.pTree.add_nodes(self.start_node)

    def set_goal(self, world_x, world_y, world_theta = 0):
        self.goal_state_map = Pose()
        goal_i, goal_j = self.world2map(world_x, world_y)
        print ("goal is %d, %d on map"%(goal_i, goal_j))
        self.goal_state_map.set_pose(goal_i, goal_j, world_theta)
        self.goal_node = prm_node(goal_i,goal_j)
        self.pTree.add_nodes(self.goal_node)

    #convert a point a map to the actual world position
    def map2world(self,mawarp_i,map_j):
        world_x = -self.graphics.environment.width/2*self.graphics.scale + map_j
        world_y = self.graphics.environment.height/2*self.graphics.scale - map_i
        return world_x, world_y

    #convert a point in world coordinate to map pixel
    def world2map(self,world_x,world_y):
        map_i = int(self.graphics.environment.width/2*self.graphics.scale - world_y)
        map_j = int(self.graphics.environment.height/2*self.graphics.scale + world_x)
        if(map_i<0 or map_i>=self.map_width or map_j<0 or map_j>=self.map_height):
            warnings.warn("Pose %f, %f outside the current map limit"%(world_x,world_y))

        if(map_i<0):
            map_i=int(0)
        elif(map_i>=self.map_width):
            map_i=self.map_width-int(1)

        if(map_j<0):
            map_j=int(0)
        elif(map_j>=self.map_height):
            map_j=self.map_height-int(1)

        return map_i, map_j

    def _init_path_img(self):
        self.map_img_np = 255*np.ones((int(self.map_width),int(self.map_height),4),dtype = np.int16)
        # self.map_img_np[0:-1][0:-1][3] = 0
        self.map_img_np[:,:,3] = 0

    def _show_path(self):
        for pose in self.path.poses:
            map_i = pose.map_i
            map_j = pose.map_j 
            self.map_img_np[map_i][map_j][1] =0
            self.map_img_np[map_i][map_j][2] =0
            self.map_img_np[map_i][map_j][3] =255

        np.savetxt("file.txt", self.map_img_np[1])

        self.path_img=Image.frombytes('RGBA', (self.map_img_np.shape[1],self.map_img_np.shape[0]), self.map_img_np.astype('b').tostring())
        # self.path_img = toimage(self.map_img_np)
        #self.path_img.show()
        self.graphics.draw_path(self.path_img)

    def check_vicinity(self,x1,y1,x2,y2,threshold = 1.0):
        if(math.sqrt((x1-x2)**2+(y1-y2)**2)<threshold):
            return True
        else:
            return False

    def plan_path(self):
        #this is the function you are going to work on
###############################################################
        st = time.time()
        #first point
        try:
            # get the bfs map
            bfsdistance = gp_to_sp_bfs(self.costmap.costmap, self.start_node.map_i,self.start_node.map_j,self.goal_node.map_i,self.goal_node.map_j) # (g) manhantann distance from start 
            
            #raious of random points
            r=100
            #ensure that end point does not lie in obstacle
            if(self.costmap.costmap[self.goal_node.map_i][self.goal_node.map_j]) == 0: #depends on how you set the value of obstacle
                print("path not possible")
            else:         
                #first random point        
                count = 0 #overall path of good nodes
                tries = 0 #nodes tried
                hit_obstacle = True
                
                while (hit_obstacle == True): #continue finding a random point that does not collide with obstacle
                    #first random point
                    ri = random.randint(self.start_node.map_i-r,self.start_node.map_i + r)
                    rj = random.randint(self.start_node.map_j-r,self.start_node.map_j + r) # Let's make a random number!
                    tries +=1
                    while ri >= self.map_width-5 or rj >= self.map_height-5:
                        ri = random.randint(self.start_node.map_i-r,self.start_node.map_i + r)
                        rj = random.randint(self.start_node.map_j-r,self.start_node.map_j + r) # Let's make a random number!
                        tries +=1
                    hit_obstacle = True
                    #print(bfsdistance[ri][rj],bfsdistance[self.start_node.map_i+1][self.start_node.map_j+1])
                    
            
                    #print(bfsdistance[ri][rj],bfsdistance[self.goal_node.map_i+1][self.goal_node.map_j+1])
                    points = bresenham(self.start_node.map_i,self.start_node.map_j,ri,rj) #generate line 
                    
                    for p in points: #check every point in line 
                        if(self.costmap.costmap[p[0]][p[1]]) == 0: #depends on how you set the value of obstacle
                            hit_obstacle = True
                            #print("we hit an obstacle",count)
                            # print ("From %d, %d to %d, %d we hit obstalce"%(self.start_node.map_i,self.start_node.map_j,ri,rj))
                            break
                        else:
                            hit_obstacle = False
                            self.path.add_pose(Pose(map_i=p[0],map_j=p[1],theta=0))
                    

                    
                    #We didn't hit an obstacle
                    if(hit_obstacle==False):
                        count +=1
                        random_node = prm_node(ri,rj)
                        self.pTree.add_nodes(random_node)
                        self.pTree.add_edges(self.start_node,random_node)#add an edge from start node to random node
                        break

            ##############################################################
            #after getting first random point start growing tree
            # get another random point and make sure that it does collide with with a precdeing line (no loops)
            # encourage path getting closer to the goal point by using brush fire distance cost 
            while True:
                r=100
                hit_obstacle = True
                while (hit_obstacle == True): #continue finding a random point that does not collide with obstacle
                    ri = random.randint(random_node.map_i-r,random_node.map_i + r)
                    rj = random.randint(random_node.map_j-r,random_node.map_j + r) # Let's make a random number!
                    tries +=1
                    while ri >= self.map_width or rj >= self.map_height: #make sure its within boundaries
                        ri = random.randint(random_node.map_i-r,random_node.map_i + r)
                        rj = random.randint(random_node.map_j-r,random_node.map_j + r) # Let's make a random number!
                        tries +=1
                        #while True:
                            #try:
                                #bfsdistance[ri][rj]
                                #break
                            #except IndexError:
                                #ri = random.randint(random_node.map_i-r,random_node.map_i + r)
                                #rj = random.randint(random_node.map_j-r,random_node.map_j + r) # Let's make a random number!
                                #tries +=1


#confirm bias with bfs distance
                    
                    
#                     while bfsdistance[ri][rj] > bfsdistance[random_node.map_i][random_node.map_j]:
#                         #print(bfsdistance[ri][rj],bfsdistance[random_node.map_i][random_node.map_j])
#                         ri = random.randint(random_node.map_i-r,random_node.map_i + r)
#                         rj = random.randint(random_node.map_j-r,random_node.map_j + r) # Let's make a random number!
#                         tries +=1
#                     
#     #restrict possible nodes to within map boundary                    
#                         while ri >= self.map_width or rj >= self.map_height:
#                             ri = random.randint(random_node.map_i-r,random_node.map_i + r)
#                             rj = random.randint(random_node.map_j-r,random_node.map_j + r) # Let's make a random number!   
#                             tries +=1
                    #print(bfsdistance[ri][rj])
                    points = bresenham(random_node.map_i,random_node.map_j,ri,rj)
                    
                    
                    
                    for p in points: #check every point in line 
                        try:
                            if(self.costmap.costmap[p[0]][p[1]]) == 0: #depends on how you set the value of obstacle
                                hit_obstacle = True
                                
                                #self.path.add_pose(Pose(map_i=p[0],map_j=p[1],theta=0))
                                #print("we hit an obstacle",count)
                                # print ("From %d, %d to %d, %d we hit obstalce"%(self.start_node.map_i,self.start_node.map_j,ri,rj))
                                break
                            else:
                                #self.path.add_pose(Pose(map_i=p[0],map_j=p[1],theta=0))
                                if(self.check_vicinity(self.goal_node.map_i,self.goal_node.map_j,p[0],p[1],5.0)): #checks if line path is within end
                                    ri = p[0]
                                    rj=p[1]
                                    
                                hit_obstacle = False
                        except IndexError:
                            hit_obstacle = True
                    
                             
                    
                    #for i in self.pTree.nodes:
                    #    if [ri,rj] == [i.map_i,i.map_j]:
                    #        hit_obstale = True
                    #        break;
                        
                             
                    #print(ri,rj)
                    if(hit_obstacle==False):
                        for p in points: #check every point in line
                            self.path.add_pose(Pose(map_i=p[0],map_j=p[1],theta=0))
                        count +=1
                        previous_node = random_node
                        random_node = prm_node(ri,rj)
                        self.pTree.add_nodes(random_node)
                        self.pTree.add_edges(previous_node,random_node)#add an edge from previous node to random node
                        break
                    
                if(self.check_vicinity(self.goal_node.map_i,self.goal_node.map_j,ri,rj,5.0)):
                #if bfsdistance[ri][rj] < 2:
                    break
                    #if bfsdistance[ri][rj] < 2:
                        #points = bresenham(ri,rj,self.goal_node.map_i,self.goal_node.map_j ) 
                    
                        #for p in points: #check every point in line 
                         #  self.path.add_pose(Pose(map_i=p[0],map_j=p[1],theta=0))
                           
                        #break;
  
                    
            #If you decide the path between start_node and random_node should be within your final path, you must do:
            #points = bresenham(random_node.map_i,random_node.map_j,self.goal_node.map_i,self.goal_node.map_j)
            #for p in points:
                #self.path.add_pose(Pose(map_i=p[0],map_j=p[1],theta=0))    
            
            #It is almost impossible for you to random a node that is coincident with goal node
            #So everytime you randomed ri and rj, you should also check if it is within a vicinity of goal
            #define check_vicinity function and decide if you have reached the goal
            #if(self.check_vicinity(self.goal_node.map_i,self.goal_node.map_j,ri,rj,2.0)):
            
            print ("We hit goal!")
            et = time.time()
            print("it took ", et-st, "seconds!")
            print("Nodes tried: ", tries)
            print("Overall node path len: ", count)
            
            points = bresenham(self.goal_node.map_i,self.goal_node.map_j,ri,rj) 
            
            
            
            #for i in range (0,count):
                #print(self.pTree.access_nodes(i))
                
                
            for p in points: #check every point in line 
                self.path.add_pose(Pose(map_i=p[0],map_j=p[1],theta=0))
                    
            self.path.save_path(file_name="Log\prm_path.csv")
            
        except KeyboardInterrupt:
            print("we have quit the path")

def gp_to_sp_bfs(grid,x1,y1,x2,y2): #educlain distance distance from goal point to start point
    #init varibles
    copylist = []
    queuelist = []
    distanceList = []
    visitedList = [] 


    #print("entering bfs")
    numRows = np.size(grid,0)
    #print(numRows)
    numCols = np.size(grid,1)
    #preload lists with default values
    for i in range(0, numRows):
        currentItem_visitlist = []
        currentItem_list = []
        currentItem_distancelist = []
        for j in range(0, numCols):
            currentItem_visitlist.append(False)
            currentItem_list.append(grid[i][j])
            currentItem_distancelist.append(0)
    

        visitedList.append(currentItem_visitlist)
        copylist.append(currentItem_list)
        distanceList.append(currentItem_distancelist)

    for i in range(0, numRows):
        for j in range(0, numCols):
            if grid[i][j] == 0:
                visitedList[i][j] = True

    #append queue from starting position find the distance 
    queuelist.append([x2,y2])
    visitedList[x1][y1] = True
    visitedList[x2][y2] = True 

    #find all occupied pixels and set them to true as well

    numChildren = len(queuelist)
    distance = 1 
    #start breadth-first-search
    while (len(queuelist)>0):
        if numChildren == 0: #increase the distance when all parents are popped out
            numChildren = len(queuelist)
            distance +=1
        #get the position value
        posx = queuelist[0][0]
        posy = queuelist[0][1]

        #pop the queue
        queuelist.pop(0)
        numChildren = numChildren -1 #subtract parent count

        if (posx-1) >= 0: #Left
            if visitedList[posx-1][posy] == False:
                visitedList[posx-1][posy] = True 
                queuelist.append([posx-1,posy])
                distanceList[posx-1][posy] = distance

            if (posy+1) < numCols: 
                if visitedList[posx-1][posy+1] == False: #top left
                    visitedList[posx-1][posy+1] = True 
                    queuelist.append([posx-1,posy+1])
                    distanceList[posx-1][posy+1] = distance+1 #diagonals needs the distnace increased by one

            if (posy-1) >= 0:
                if visitedList[posx-1][posy-1] == False: # bottom left
                    visitedList[posx-1][posy-1] = True 
                    queuelist.append([posx-1,posy-1])
                    distanceList[posx-1][posy-1] = distance+1 #diagonals needs the distnace increased by one
                    
        if posx+1 < numRows: #right
            if visitedList[posx+1][posy] == False:
                visitedList[posx+1][posy] = True
                queuelist.append([posx+1,posy])
                distanceList[posx+1][posy] = distance
               

            if (posy+1) < numCols: 
                if visitedList[posx+1][posy+1] == False: #top right
                    visitedList[posx+1][posy+1] = True 
                    queuelist.append([posx+1,posy+1])
                    distanceList[posx+1][posy+1] = distance+1 #diagonals needs the distnace increased by one

            if (posy-1) >= 0:
                if visitedList[posx+1][posy-1] == False: # bottom right
                    visitedList[posx+1][posy-1] = True 
                    queuelist.append([posx+1,posy-1])
                    distanceList[posx+1][posy-1] = distance+1 #diagonals needs the distnace increased by one
                    
        if posy-1 >= 0: #bottom
            if visitedList[posx][posy-1] == False:
                visitedList[posx][posy-1] = True
                queuelist.append([posx,posy-1])
                distanceList[posx][posy-1] = distance

        if posy+1 < numCols: #top
            if visitedList[posx][posy+1] == False:
                visitedList[posx][posy+1] = True
                queuelist.append([posx,posy+1])
                distanceList[posx][posy+1] = distance


    np.savetxt("Log/distancemapgptosp.txt",np.array(distanceList))     
    return distanceList

# bresenham algorithm for line generation on grid map
# from http://www.roguebasin.com/index.php?title=Bresenham%27s_Line_Algorithm
def bresenham(x1, y1, x2, y2):
    dx = x2 - x1
    dy = y2 - y1
 
    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)
 
    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2
 
    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True
 
    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1
 
    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1
 
    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx
 
    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()

    # print points
    return points

