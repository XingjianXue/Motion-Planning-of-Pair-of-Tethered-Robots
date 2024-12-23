import numpy as np
from obstacle_class import Obstacle
from tether_update import distance, tether_update,tether_update_single,tether_not_cross,tether_winding_angle
from shapely.geometry import Polygon, LineString, Point
from queue import PriorityQueue 
from tangent_node import tangent_node_, tangent_node,get_tangent_node,get_goal,get_goal_from_start,obstacle,distance_sum,rearrange
import copy as cp
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as P
from matplotlib.patches import Circle
from shapely import is_simple
import time
from algorithm import tether_hybrid_Astar1,tether_hybrid_Astar2,is_the_input_valid,Robot_pair,Robot, g_cost,heuristic_cost,list2tuple,winding_constraint_check,winding_Astar,dubinsPath1,dubinsPath2
from plot import plot

#change the start pose and tether configuration here
start_pos = [[0,0,np.pi/4],[20,0,np.pi/2]]
tether_config = [start_pos[0][0:2],start_pos[1][0:2]]

# start_pos = [[5.0, 26.0, np.pi/2],[20.9762, 17.7169, -np.arctan2(3,1)]]
# tether_config = [[5.0, 26.0], [6.7, 21], [6.1, 10], [6, 5], [12, 5], [15, 22], [19, 22], [20.9762, 17.7169]]

#change the goal pose and tether configuration here
goal_pos = [[5,26,np.pi/2],[18,26,np.pi/2]]

#change the winding constraint here
winding_constraint = 1.5*np.pi

#change the simulation step here
step_n = 0.3

#curvature constraint: r = 1, do not change this, otherwise it will lead to errors.
r = 1
length_limit = 300
index = 0
start_time = time.time() 
robot_pair =  winding_Astar(start_pos,goal_pos,tether_config,obstacle, winding_constraint)
# print(robot_pair)
# cost_sum = robot_pair.winding

time_count = time.time() - start_time
print("--- %s seconds ---" % (time.time() - start_time)) 

tether_config_list = []
robot_pair_list = []
while True:
    tether_config_list.append(robot_pair.tether_config)
    robot_pair_list.append(robot_pair)
    robot_pair = robot_pair.parent
    # print(robot_pair.tether_config)
    # print(robot_pair.transit1)
    # print(robot_pair.transit2)
    if robot_pair.parent == None:
        tether_config_list.append(robot_pair.tether_config)
        robot_pair_list.append(robot_pair)            
        break
tether_config_list = tether_config_list[::-1]
tether_config = tether_config_list[-1]
robot_pair_list = robot_pair_list[::-1]
# for i in range(1,len(robot_pair_list)):
#     print("############################")
#     print(robot_pair_list[i].parent.tangent_node1.center, robot_pair_list[i].parent.tangent_node2.center)
#     print(robot_pair_list[i].parent.tether_config)
#     print(robot_pair_list[i].transit1,robot_pair_list[i].transit2)
#     print(robot_pair_list[i].tether_config)
# for i in tether_config_list:
#     print(i)

# obstacle_points = []
# for i in obstacle:
#     for j in i:
#         obstacle_points.append(j)   
# fig,ax = plt.subplots()
# ax.set_xlim([-5,30])
# ax.set_ylim([-5,30])
# for i in obstacle:
#     p = P(i, facecolor = 'k')
#     ax.add_patch(p)
# for point in obstacle_points:
#     circle = Circle((point[0],point[1]), 1,color = "k", fill = False, linestyle = "--")
#     ax.add_patch(circle)
# plt.pause(5)
# for tether_config in tether_config_list:
#     tether_plot = ax.plot([i[0] for i in tether_config],[i[1] for i in tether_config],'r')
#     plt.pause(1)
#     if tether_config != tether_config_list[-1]:
#         line = tether_plot.pop(0)
#         line.remove()



# for i in range(1,len(robot_pair_list)):
#     # print("############################")
#     # print(robot_pair_list[i].parent.tangent_node1.center, robot_pair_list[i].parent.tangent_node2.center)
#     # print(robot_pair_list[i].parent.tether_config)
#     # print(robot_pair_list[i].transit1,robot_pair_list[i].transit2)
#     # print(robot_pair_list[i].tether_config)
#     parent_center1 = robot_pair_list[i].parent.tangent_node1.center
#     parent_center2 = robot_pair_list[i].parent.tangent_node2.center
#     parent_tether_config = robot_pair_list[i].parent.tether_config
#     tether_config = robot_pair_list[i].tether_config
#     transit1 = robot_pair_list[i].transit1
#     transit2 = robot_pair_list[i].transit2
#     if i == 1:    
#         if abs(distance(parent_center1[0], transit1) - 1) < 0.01:
#             parent_center1 = parent_center1[0]
#         else:
#             parent_center1 = parent_center1[1]  
#         if abs(distance(parent_center2[0], transit2) - 1) < 0.01:
#             parent_center2 = parent_center2[0]
#         else:
#             parent_center2 = parent_center2[1] 
obstacle_class_list = []
obstacle_points = []
for i in obstacle:
    obstacle_class_list.append(Obstacle(i))
    for j in i:
        obstacle_points.append(j)      

    
         
fig,ax = plt.subplots()
ax.set_xlim([-5,30])
ax.set_ylim([-5,30])
# plt.title("Winding Constraint: "+ str(np.round(winding_constraint,3))+" rad")
for i in obstacle:
    p = P(i, facecolor = 'k')
    ax.add_patch(p)
for point in obstacle_points:
    circle = Circle((point[0],point[1]), 1,color = "grey", fill = False, linewidth = 0.3)
    ax.add_patch(circle) 
    
tangent_line_list = []       
for point in obstacle_points:
    # print(point)
    point = [point[0]+1, point[1]]
    tangent_line_list = [*tangent_line_list, *tangent_node_([point,point],point,goal_pos[0],start_pos[0],obstacle).neighbor]  
    tangent_line_list = [*tangent_line_list, *tangent_node_([point,point],point,goal_pos[1],start_pos[1],obstacle).neighbor] 
    point = [point[0]-2, point[1]]
    tangent_line_list = [*tangent_line_list, *tangent_node_([point,point],point,goal_pos[0],start_pos[0],obstacle).neighbor]  
    tangent_line_list = [*tangent_line_list, *tangent_node_([point,point],point,goal_pos[1],start_pos[1],obstacle).neighbor] 
tangent_line_list = [*tangent_line_list, *tangent_node_(start_pos[0],None,goal_pos[0],start_pos[0],obstacle).neighbor]
tangent_line_list = [*tangent_line_list, *tangent_node_(start_pos[1],None,goal_pos[1],start_pos[1],obstacle).neighbor]
for tangent_line in tangent_line_list:
    ax.plot([i[0] for i in tangent_line], [i[1] for i in tangent_line], color = 'grey', linewidth = 0.2)


ax.arrow(start_pos[0][0], start_pos[0][1], 0.2*np.cos(start_pos[0][2]), 0.2*np.sin(start_pos[0][2]), head_width = 1, width = 0.05, color = "r")
ax.arrow(start_pos[1][0], start_pos[1][1], 0.2*np.cos(start_pos[1][2]), 0.2*np.sin(start_pos[0][2]), head_width = 1, width = 0.05, color = "r")
ax.arrow(goal_pos[0][0], goal_pos[0][1], 0.2*np.cos(goal_pos[0][2]), 0.2*np.sin(goal_pos[0][2]), head_width = 1, width = 0.05, color = "b")
ax.arrow(goal_pos[1][0], goal_pos[1][1], 0.2*np.cos(goal_pos[1][2]), 0.2*np.sin(goal_pos[1][2]), head_width = 1, width = 0.05, color = "b")
start1 = Circle((start_pos[0][0], start_pos[0][1]), 0.2, color = 'r', fill = True)
start2 = Circle((start_pos[1][0], start_pos[1][1]), 0.2, color = 'r', fill = True)
goal1 = Circle((goal_pos[0][0], goal_pos[0][1]), 0.2, color = 'b', fill = True)
goal2 = Circle((goal_pos[1][0], goal_pos[1][1]), 0.2, color = 'b', fill = True)
ax.add_patch(start1)
ax.add_patch(start2)
ax.add_patch(goal1)
ax.add_patch(goal2)
first_robot = robot_pair_list[0]
center1 = first_robot.tangent_node1.center[0]
center2 = first_robot.tangent_node1.center[1]
center3 = first_robot.tangent_node2.center[0]
center4 = first_robot.tangent_node2.center[1]
center5 = first_robot.tangent_node1.goal_center1
center6 = first_robot.tangent_node1.goal_center2
center7 = first_robot.tangent_node2.goal_center1
center8 = first_robot.tangent_node2.goal_center2
start_circle1 = Circle((center1[0], center1[1]), r, color = "grey", fill = False, linewidth = 0.3)    
start_circle2 = Circle((center2[0], center2[1]), r, color = "grey", fill = False, linewidth = 0.3) 
start_circle3 = Circle((center3[0], center3[1]), r, color = "grey", fill = False, linewidth = 0.3)    
start_circle4 = Circle((center4[0], center4[1]), r, color = "grey", fill = False, linewidth = 0.3) 
goal_circle1 = Circle((center5[0], center5[1]), r, color = "grey", fill = False, linewidth = 0.3)    
goal_circle2 = Circle((center6[0], center6[1]), r, color = "grey", fill = False, linewidth = 0.3) 
goal_circle3 = Circle((center7[0], center7[1]), r, color = "grey", fill = False, linewidth = 0.3)    
goal_circle4 = Circle((center8[0], center8[1]), r, color = "grey", fill = False, linewidth = 0.3) 
ax.add_patch(start_circle1)
ax.add_patch(start_circle2) 
ax.add_patch(start_circle3)
ax.add_patch(start_circle4) 
ax.add_patch(goal_circle1)
ax.add_patch(goal_circle2) 
ax.add_patch(goal_circle3)
ax.add_patch(goal_circle4) 
plt.pause(5)
# print(robot_pair_list[-1].transit1)   


for i in range(1,len(robot_pair_list)):

    tag = i
    robot_pair = robot_pair_list[i]

    tether_config = robot_pair.tether_config 
    # print(tether_config)     
    robot1 = robot_pair.tangent_node1       
    pos1 = robot1.point
    parent1 = robot1.parent
    parent_center1 = robot1.parent_center
    # parent_angle_range1 = robot1.parent_angle_range
    
    transit1 = robot1.transit
    # print(transit1)
    vect1 = np.array(transit1)-np.array(parent_center1)     
    transit1_angle = np.arctan2(vect1[1],vect1[0])
    vect1_ = np.array(parent1)-np.array(parent_center1)
    parent1_angle = np.arctan2(vect1_[1],vect1_[0])
    
    robot2 = robot_pair.tangent_node2
    pos2 = robot2.point
    parent2 = robot2.parent
    parent_center2 = robot2.parent_center
    # parent_angle_range2 = robot2.parent_angle_range
    transit2 = robot2.transit
    vect2 = np.array(transit2)-np.array(parent_center2)
    transit2_angle = np.arctan2(vect2[1], vect2[0])
    vect2_ = np.array(parent2)-np.array(parent_center2)
    parent2_angle = np.arctan2(vect2_[1],vect2_[0])
    
    # parent_center1 = robot_pair_list[i].parent.tangent_node1.center
    # parent_center2 = robot_pair_list[i].parent.tangent_node2.center
    # parent_tether_config = robot_pair_list[i].parent.tether_config
    # tether_config = robot_pair_list[i].tether_config
    # transit1 = robot_pair_list[i].transit1
    # transit2 = robot_pair_list[i].transit2
    # print(parent_center1, parent_center2)
    # if i == 1:    
    #     if abs(distance(parent_center1[0], transit1) - 1) < 0.01:
    #         parent_center1 = parent_center1[0]
    #     else:
    #         parent_center1 = parent_center1[1]  
    #     if abs(distance(parent_center2[0], transit2) - 1) < 0.01:
    #         parent_center2 = parent_center2[0]
    #     else:
    #         parent_center2 = parent_center2[1] 
        
    angle1_range = []
    line1_range = []
    angle2_range = []          
    line2_range = []
    trajectory1 = []
    trajectory2 = []
    
    # print("################")
    # print("pos2")
    # print(pos2)
    # print("transit2")
    # print(transit2)
    # print("parent2")
    # print(parent2)
    # print("parent_center2")
    # print(parent_center2)
    # print("parent2_angle")
    # print(parent2_angle)
    # print(transit2_angle)
    
    # print("################")
    # print("pos1")
    # print(pos1)
    # print("transit1")
    # print(transit1)
    # print("parent1")
    # print(parent1)
    # print("parent_center1")
    # print(parent_center1)
    # print("parent1_angle")
    # print(parent1_angle)
    # print("transit2_angle")
    # print(transit1_angle)
    # if distance(parent1, goal1[0:2]) < 2:
    #     print(1)
    
    # print(pos1,pos2)
    if max(transit1_angle,parent1_angle) - min(transit1_angle,parent1_angle) <= np.pi:
        # print(1)
        vector_from_transit_to_neighbor = np.array(pos1) - np.array(transit1)
        distance1 = np.linalg.norm(vector_from_transit_to_neighbor)
        num_step1 = int(distance1/step_n) + 1
        for i in range(0,num_step1):
            line_increment = (np.array(pos1) - np.array(transit1))*i/num_step1
            line = np.array(transit1) + np.array(line_increment)
            line1_range.append(list(line)) 
        distance2 = abs((transit1_angle - parent1_angle)*r)
        num_step2 = int(distance2/step_n) + 1     
        # print(transit1_angle,parent1_angle)           
        for i in range(0,num_step2):
            increment = (transit1_angle - parent1_angle)*i/num_step2
            angle = np.array(parent1_angle) + np.array(increment)
            angle1_range.append(angle)
        arc_pos1 = []
        for angle in angle1_range:
            p = r*np.array([np.cos(angle), np.sin(angle)]) + np.array(parent_center1)
            arc_pos1.append(list(p))
        trajectory1 = []
        trajectory1 = [*arc_pos1, *line1_range]
        # print("#########################")
        # print(pos1)
        # print(transit1)
        # print(parent1)
        # print(parent_center1)
        # print(arc_pos1)
        # print(line1_range)
        arc_pos1 = []
        line1_range = []
        # if distance(parent1, goal1[0:2]) < 2:
        #     print(1)
    

    if max(transit1_angle,parent1_angle) - min(transit1_angle,parent1_angle) > np.pi: 
        if transit1_angle >= parent1_angle:
            # print(2)
            parent1_angle = parent1_angle + 2*np.pi
            vector_from_transit_to_neighbor = np.array(pos1) - np.array(transit1)
            distance1 = np.linalg.norm(vector_from_transit_to_neighbor)
            num_step1 = int(distance1/step_n) + 1
            for i in range(0,num_step1):
                line_increment = (np.array(pos1) - np.array(transit1))*i/num_step1
                line = np.array(transit1) + np.array(line_increment)
                line1_range.append(list(line)) 
            distance2 = abs((transit1_angle - parent1_angle)*r)
            num_step2 = int(distance2/step_n) + 1                
            for i in range(0,num_step2):
                increment = (transit1_angle - parent1_angle)*i/num_step2
                angle = np.array(parent1_angle) + np.array(increment)
                angle1_range.append(angle)
            arc_pos1 = []
            for angle in angle1_range:
                p = r*np.array([np.cos(angle), np.sin(angle)]) + np.array(parent_center1)
                arc_pos1.append(list(p))
            trajectory1 = []
            trajectory1 = [*arc_pos1, *line1_range] 
            arc_pos1 = []
            line1_range = []
            parent1_angle = parent1_angle - 2*np.pi
            # print(len(trajectory1))                   
            
        if transit1_angle < parent1_angle:
            # print(3)
            transit1_angle = transit1_angle + 2*np.pi
            
            vector_from_transit_to_neighbor = np.array(pos1) - np.array(transit1)
            distance1 = np.linalg.norm(vector_from_transit_to_neighbor)
            num_step1 = int(distance1/step_n) + 1
            for i in range(0,num_step1):
                line_increment = (np.array(pos1) - np.array(transit1))*i/num_step1
                line = np.array(transit1) + np.array(line_increment)
                line1_range.append(list(line)) 
            distance2 = abs((transit1_angle - parent1_angle)*r)
            num_step2 = int(distance2/step_n) + 1                
            for i in range(0,num_step2):
                increment = (transit1_angle - parent1_angle)*i/num_step2
                angle = np.array(parent1_angle) + np.array(increment)
                angle1_range.append(angle)
                
            arc_pos1 = []
            for angle in angle1_range:
                p = r*np.array([np.cos(angle), np.sin(angle)]) + np.array(parent_center1)
                arc_pos1.append(list(p))
            trajectory1 = []
            trajectory1 = [*arc_pos1, *line1_range]
            arc_pos1 = []
            line1_range = []
            transit1_angle = transit1_angle - 2*np.pi

            
    # print(transit2_angle,parent2_angle)
    # print(max(transit2_angle,parent2_angle) - min(transit2_angle,parent2_angle) <= np.pi)                   
    if max(transit2_angle,parent2_angle) - min(transit2_angle,parent2_angle) <= np.pi:
        # print('a')
        vector_from_transit_to_neighbor2 = np.array(pos2) - np.array(transit2)
        distance1 = np.linalg.norm(vector_from_transit_to_neighbor2)
        num_step1 = int(distance1/step_n) + 1
        for i in range(0,num_step1):
            line_increment = (np.array(pos2) - np.array(transit2))*i/num_step1
            line = np.array(transit2) + np.array(line_increment)
            line2_range.append((list(line))) 
        distance2 = abs((transit2_angle - parent2_angle)*r)
        num_step2 = int(distance2/step_n) + 1    
        for i in range(0,num_step2):
            increment = (transit2_angle - parent2_angle)*i/num_step2
            angle = np.array(parent2_angle) + np.array(increment)
            angle2_range.append(angle)
        arc_pos2 = []
        for angle in angle2_range:
            p = r*np.array([np.cos(angle), np.sin(angle)]) + np.array(parent_center2)
            arc_pos2.append(list(p))
        trajectory2 = []
        trajectory2 = [*arc_pos2, *line2_range]
        line2_range = []
        arc_pos2 = []
        
    if max(transit2_angle,parent2_angle) - min(transit2_angle,parent2_angle) > np.pi:  
        if transit2_angle >= parent2_angle:
            # print("b")
            parent2_angle = parent2_angle + 2*np.pi
            vector_from_transit_to_neighbor2 = np.array(pos2) - np.array(transit2)
            distance1 = np.linalg.norm(vector_from_transit_to_neighbor2)
            num_step1 = int(distance1/step_n) + 1
            for i in range(0,num_step1):
                line_increment = (np.array(pos2) - np.array(transit2))*i/num_step1
                line = np.array(transit2) + np.array(line_increment)
                line2_range.append((list(line))) 
            distance2 = abs((transit2_angle - parent2_angle)*r)
            num_step2 = int(distance2/step_n) + 1    
            for i in range(0,num_step2):
                increment = (transit2_angle - parent2_angle)*i/num_step2
                angle = np.array(parent2_angle) + np.array(increment)
                angle2_range.append(angle)
            arc_pos2 = []
            for angle in angle2_range:
                p = r*np.array([np.cos(angle), np.sin(angle)]) + np.array(parent_center2)
                arc_pos2.append(list(p))
            trajectory2 =[]
            trajectory2 = [*arc_pos2, *line2_range] 
            line2_range = []
            arc_pos2 = [] 
            parent2_angle = parent2_angle - 2*np.pi
    
            
        if transit2_angle < parent2_angle :
            # print('c')
            transit2_angle = transit2_angle + 2*np.pi
            vector_from_transit_to_neighbor2 = np.array(pos2) - np.array(transit2)
            distance1 = np.linalg.norm(vector_from_transit_to_neighbor2)
            num_step1 = int(distance1/step_n) + 1
            for i in range(0,num_step1):
                line_increment = (np.array(pos2) - np.array(transit2))*i/num_step1
                line = np.array(transit2) + np.array(line_increment)
                line2_range.append((list(line))) 
            distance2 = abs((transit2_angle - parent2_angle)*r)
            num_step2 = int(distance2/step_n) + 1    
            for i in range(0,num_step2):
                increment = (transit2_angle - parent2_angle)*i/num_step2
                angle = np.array(parent2_angle) + np.array(increment)
                angle2_range.append(angle)
            arc_pos2 = []
            for angle in angle2_range:
                p = r*np.array([np.cos(angle), np.sin(angle)]) + np.array(parent_center2)
                arc_pos2.append(list(p))
            trajectory2 = []
            trajectory2 = [*arc_pos2, *line2_range]
            line2_range = []
            arc_pos2 = []
            transit2_angle = parent2_angle - 2*np.pi                   
    # print('trajectory1')
    # print(trajectory1)
    # print('trajectory2')
    # print(trajectory2)
    # print(trajectory1)
    # print(trajectory2)
    if len(trajectory1) > len(trajectory2):
        num = len(trajectory1) - len(trajectory2)
        for i in range(0,num):
            trajectory2.append(pos2)
    if len(trajectory2) > len(trajectory1):
        num = len(trajectory2) - len(trajectory1)
        for i in range(0,num):
            trajectory1.append(pos1)
    tether_config = tether_config_list[tag - 1]
    # print(trajectory1)
    # print(trajectory2)
    for i in range(0,len(trajectory1)-1):
        tether_config, length = tether_update([trajectory1[i], trajectory2[i]], [trajectory1[i+1], trajectory2[i+1]], tether_config, obstacle)
        # print(tether_config)
        # print(tether_config)
        tether_plot = ax.plot([j[0] for j in tether_config],[j[1] for j in tether_config],'r')    
        path1 = Circle((tether_config[0][0], tether_config[0][1]), 0.05, color = 'g', fill = True)
        path2 = Circle((tether_config[-1][0], tether_config[-1][1]), 0.05, color = 'c', fill = True)
        # r1 = Circle((tether_config[0][0], tether_config[0][1]), 0.5, color = 'r', fill = True)
        # r2 = Circle((tether_config[-1][0], tether_config[-1][1]), 0.5, color = 'r', fill = True)              
        ax.add_patch(path1)
        ax.add_patch(path2)
        # ax.add_patch(r1)
        # ax.add_patch(r2) 
                    
        plt.pause(0.001)  
        line = tether_plot.pop(0)
        line.remove()
tether_plot = ax.plot([i[0] for i in tether_config],[i[1] for i in tether_config],'r')

# name_save = "Winding_Constraint" + str(np.round(winding_constraint,3)) +"cost" + str(np.round(robot_pair_list[-1].cost,3))+".jpg"
# plt.savefig(name_save) 
plt.show() 



            
  