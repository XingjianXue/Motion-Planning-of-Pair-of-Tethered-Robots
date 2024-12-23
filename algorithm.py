# Author: Xingjian Xue
# The algorithm has two parts, which can be simulated respectively in main.py and main2.py 
# The first part does not involve the winding angles and the robots are only required to find their goal
# The second part adds the winding constraint and unlike the first part, the robot can only move in one direction

# problem to be solved
    # The estimated total cost and the fianl calculated total cost has small difference, which should be corrected.
    # The result is not the optimal even when the winding constraint is removed, which is strange.
    

import numpy as np
from obstacle_class import Obstacle
from tether_update import distance, tether_update,tether_update_single,tether_not_cross,tether_winding_angle,tether_not_cross_winding
from shapely.geometry import Polygon, LineString, Point
from queue import PriorityQueue 
from tangent_node import tangent_node_, tangent_node,get_tangent_node,get_goal,get_goal_from_start,obstacle,distance_sum,rearrange,get_goal_
import copy as cp
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as P
from matplotlib.patches import Circle
from shapely import is_simple
import time

class Robot_pair:
        def __init__(self,tangent_node1, tangent_node2, tether_config, parent = None):
            self.tangent_node1 = tangent_node1
            self.tangent_node2 = tangent_node2
            self.tether_config = tether_config
            self.parent = parent
            self.num = 2
            
class Robot:
    def __init__(self,tangent_node, tether_config, parent = None):
        self.tangent_node = tangent_node
        self.tether_config = tether_config
        self.parent = parent
        self.num = 1
    
        
def g_cost(tangent_node, transit, neighbor):
    # print(tangent_node.center)
    r = 1
    # print(tangent_node.center[0])
    # print(type(tangent_node.center[0]) == int)
    if type(tangent_node.center[0]) != list:
        r = 1
        v1 = np.array(tangent_node.point) - np.array(tangent_node.center)
        v2 = np.array(transit) - np.array(tangent_node.center)
        # print("############")
        #add from 1-2
        if np.linalg.norm(v1)*np.linalg.norm(v2) != 0:
            dot_product = np.dot(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2))
            if dot_product > 1:
                dot_product = 1
            arc = np.arccos(dot_product)
            # print(np.dot(v1,v2))
            # print(np.linalg.norm(v1)*np.linalg.norm(v2))       
            # print(v1)
            # print(v2)
            # print(arc)
            cost = abs(arc*r) + distance(transit, neighbor)
            # print(arc*r)
            # print(distance(transit, neighbor))
            return cost     
        else:
            return 0   
    if type(tangent_node.center[0]) == list: 
        if abs(distance(transit, tangent_node.center[0]) - r) < 0.01:
            center = tangent_node.center[0]
            r = 1
            v1 = np.array(tangent_node.point) - np.array(center)
            v2 = np.array(transit) - np.array(center)
            # print("############")
            #add from 1-2
            if np.linalg.norm(v1)*np.linalg.norm(v2) != 0:
                dot_product = np.dot(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2))
                if dot_product > 1:
                    dot_product = 1
                arc = np.arccos(dot_product)
                # print(np.dot(v1,v2))
                # print(np.linalg.norm(v1)*np.linalg.norm(v2))       
                # print(v1)
                # print(v2)
                # print(arc)
                cost = abs(arc*r) + distance(transit, neighbor)
                # print(arc*r)
                # print(distance(transit, neighbor))
                return cost  
        if abs(distance(transit, tangent_node.center[1]) - r) < 0.01:
            center = tangent_node.center[1]
            r = 1
            v1 = np.array(tangent_node.point) - np.array(center)
            v2 = np.array(transit) - np.array(center)
            # print("############")
            #add from 1-2
            if np.linalg.norm(v1)*np.linalg.norm(v2) != 0:
                dot_product = np.dot(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2))
                if dot_product > 1:
                    dot_product = 1
                arc = np.arccos(dot_product)
                # print(np.dot(v1,v2))
                # print(np.linalg.norm(v1)*np.linalg.norm(v2))       
                # print(v1)
                # print(v2)
                # print(arc)
                cost = abs(arc*r) + distance(transit, neighbor)
                # print(arc*r)
                # print(distance(transit, neighbor))
                return cost 
            
            
        
def heuristic_cost(current_node, goal):
    return distance(current_node, goal)

def list2tuple(tether_config):
    tuplelist = []
    for i in tether_config:
        tuplelist.append(tuple(np.round(i,2)))
    return tuple(tuplelist)

def tether_hybrid_Astar1(start_pos,goal_pos,tether_config,length_limit, obstacle):
    start1 = start_pos[0]
    start2 = start_pos[1]
    goal1 = goal_pos[0]
    goal2 = goal_pos[1]
    tangent_node1 = tangent_node(start1,None,goal1,start1)
    tangent_node2 = tangent_node(start2,None,goal2,start2)
    
    goal_neighbor1 = tangent_node1.neighbor[-1][1]
    transit1 = tangent_node1.neighbor[-1][0]
    goal_neighbor2 = tangent_node2.neighbor[-1][1]
    transit2 = tangent_node2.neighbor[-1][0]
    if distance(goal_neighbor1, goal1) < 0.01 and distance(goal_neighbor2, goal2) < 0.01:
        start_robot_pair = Robot_pair(tangent_node1, tangent_node2, tether_config, parent = None)
        new_cost = g_cost(tangent_node1,transit1, goal_neighbor1) + g_cost(tangent_node2,transit2, goal_neighbor2)
        print("distance travel in the first part")
        print(new_cost)
        new_tether_config, length = tether_update([tangent_node1.point, tangent_node2.point], [goal_neighbor1, goal_neighbor2],tether_config, obstacle)                  
        if length < length_limit:
            if tether_not_cross(new_tether_config) == True:
                new_tangent_node1 = tangent_node(tangent_node1.neighbor[-1],tangent_node1.point,goal1,start1)
                new_tangent_node2 = tangent_node(tangent_node2.neighbor[-1],tangent_node2.point,goal2,start2)
                New_robot_pair = Robot_pair(new_tangent_node1,new_tangent_node2,new_tether_config,start_robot_pair)
                New_robot_pair.cost = new_cost
                return New_robot_pair
        
        
    
    cost_so_far = {}
    cost_so_far[list2tuple(tether_config)] = 0       
    tag = 0
    openlist = PriorityQueue()
    first_robot_pair = Robot_pair(tangent_node1, tangent_node2, tether_config, parent = None)
    first_robot_pair.cost = 0
    # openlist.put((0,tag,Robot_pair(tangent_node1, tangent_node2, tether_config, parent = None)))
    openlist.put((0,tag,first_robot_pair))
    
    while not openlist.empty():
        # print(1)
        extract = openlist.get()
        # print(extract)
        current_robot_pair = extract[2]
        tc = extract[0]
        # current_robot_pair = openlist.get()[2]
        robot1 = current_robot_pair.tangent_node1
        robot2 = current_robot_pair.tangent_node2
        tether_config = current_robot_pair.tether_config
        current_cost_to_go = current_robot_pair.cost
        # print(robot1.point)
        # print(robot2.point)
        # print("##############")
        # print("tether_config")
        # print(tether_config)
        # print("total cost")
        # print(tc)
        
        if distance(robot1.point, goal1) < 0.01 or distance(robot2.point, goal2) < 0.01:
            # print("tether_config from the first part")
            # print(tether_config)
            print("distance travel in the first part")
            print(current_robot_pair.cost)
            return current_robot_pair
        neighbor_list1 = robot1.neighbor
        neighbor_list2 = robot2.neighbor
        for neighbor1 in neighbor_list1:
            # print("################")
            # print(robot1.point)
            # print(neighbor1[0])
            # print(neighbor1[1])
            # print(g_cost(robot1,neighbor1[0], neighbor1[1]))
            # time.sleep(1)
            for neighbor2 in neighbor_list2:                     
                # print("################")
                # print(robot2.point)
                # print(neighbor2[0])
                # print(neighbor2[1])
                # print(g_cost(robot2,neighbor2[0], neighbor2[1]))
                # time.sleep(0.1)
                # print(cost_so_far)
                new_cost = current_cost_to_go + g_cost(robot1,neighbor1[0], neighbor1[1]) + g_cost(robot2,neighbor2[0], neighbor2[1])                                    
                # new_cost = cost_so_far[list2tuple(tether_config)] + g_cost(robot1,neighbor1[0], neighbor1[1]) + g_cost(robot2,neighbor2[0], neighbor2[1])
                # print("################")   
                # print(g_cost(robot1,neighbor1[0], neighbor1[1]))  
                # print(g_cost(robot2,neighbor2[0], neighbor2[1]))     
                # print(current_cost_to_go )   
                # print(new_cost)   
                # time.sleep(0.1)   
                old_tether = tether_config 
                new_tether_config, length = tether_update([robot1.point, robot2.point], [neighbor1[1], neighbor2[1]],tether_config, obstacle)                  
                
                if length < length_limit:
                    if tether_not_cross(new_tether_config) == True:                        
                        if list2tuple(new_tether_config) not in cost_so_far or new_cost < cost_so_far[list2tuple(new_tether_config)]:
                            # try:
                            #     print(new_cost < cost_so_far[list2tuple(new_tether_config)])
                            # except:
                            #     print("go on")
                            # print(cost_so_far[list2tuple(new_tether_config)])
                            # print(new_cost)
                            cost_so_far[list2tuple(new_tether_config)] = new_cost
                            total_cost = new_cost + heuristic_cost(neighbor1[1], goal1)+ heuristic_cost(neighbor2[1], goal2)
                            # print("################")
                            # print(heuristic_cost(neighbor2[1], goal2))
                            # print(heuristic_cost(neighbor1[1], goal1))
                            
                            # print(total_cost)
                            # print(new_cost)
                            new_tangent_node1 = tangent_node(neighbor1,robot1.point,goal1,start1)
                            new_tangent_node2 = tangent_node(neighbor2,robot2.point,goal2,start2)
                            New_robot_pair = Robot_pair(new_tangent_node1,new_tangent_node2,new_tether_config,current_robot_pair)
                            New_robot_pair.cost = new_cost
                            tag = tag+1
                            # print(length)
                            # if distance(new_tangent_node1.point,goal1) < 0.01 or distance(new_tangent_node2.point,goal2) < 0.01:
                            #     # print(New_robot_pair.tether_config)
                            #     # print("distance travel in the first part")
                            #     # print(new_cost)
                            #     # return New_robot_pair
                            #     print("######")
                            #     print(old_tether)
                            #     print(new_tether_config)
                            #     print(total_cost)
                            #     print(new_cost)
                            #     print(heuristic_cost(neighbor1[1], goal1))
                            #     print(heuristic_cost(neighbor2[1], goal2))
                            #     # print(length)
                            #     print("######")
                            openlist.put((total_cost,tag,New_robot_pair))
    return False

def tether_hybrid_Astar2(goal_pos,current_robot_pair,length_limit, obstacle, start_pos):
    goal1 = goal_pos[0]
    goal2 = goal_pos[1]
    tether_config = current_robot_pair.tether_config
    if distance(tether_config[0], goal1) < 0.01 and distance(tether_config[-1], goal2) < 0.01:
        index = 0
        return current_robot_pair,index
    
    if distance(tether_config[0], goal1) < 0.01:
        robot = Robot(current_robot_pair.tangent_node2, tether_config, current_robot_pair)
        goal = goal2         
        reverse = True   
        start = start_pos[1]
        index = 1
        robot.num = -1
    if distance(tether_config[-1], goal2) < 0.01:
        robot = Robot(current_robot_pair.tangent_node1, tether_config, current_robot_pair)
        goal = goal1
        reverse = False
        start = start_pos[0]
        index = 0
        robot.num = -1

    
    cost_so_far = {}
    cost_so_far[list2tuple(tether_config)] = 0
    
    tag = 0
    openlist = PriorityQueue()
    robot.cost = 0
    openlist.put((0,tag,robot))
    
    while not openlist.empty():
        # print(2)
        current_robot = openlist.get()[2]
        robot = current_robot.tangent_node
        tether_config = current_robot.tether_config
        current_cost_to_go = current_robot.cost
        # print(robot.point)
        if distance(robot.point, goal) < 0.01:
            print("distance travel in the second part")
            print(current_robot.cost)
            # print("total length")
            # print(length)                
            return current_robot, index

        neighbor_list = robot.neighbor
        for neighbor in neighbor_list:                
                new_cost = current_cost_to_go + g_cost(robot,neighbor[0], neighbor[1])
                # new_cost = cost_so_far[list2tuple(tether_config)] + g_cost(robot,neighbor[0], neighbor[1])
                new_tether_config, length = tether_update_single(robot.point, neighbor[1],tether_config, obstacle,reverse)                  
                if length < length_limit:
                    if tether_not_cross(new_tether_config) == True:  
                        if list2tuple(new_tether_config) not in cost_so_far or new_cost < cost_so_far[list2tuple(new_tether_config)]:
                            cost_so_far[list2tuple(new_tether_config)] = new_cost
                            total_cost = new_cost + heuristic_cost(neighbor[1], goal)
                            new_tangent_node = tangent_node(neighbor,robot.point,goal,start)
                            New_robot = Robot(new_tangent_node,new_tether_config,current_robot)
                            New_robot.cost = new_cost
                            tag = tag + 1
                            # print(length)
                            # if distance(new_tangent_node.point, goal) < 0.01:
                            #     print("distance travel in the second part")
                            #     print(new_cost)
                            #     print("total length")
                            #     print(length)
                            #     return New_robot, index
                            openlist.put((total_cost,tag,New_robot))
    return False











def reform(tether_config,obstacle_points):
    # print(tether_config)
    r = 1
    first = tether_config[0]
    second = tether_config[-1]
    middle = tether_config[1:-1]
    # print(first,second,middle)
    for point in obstacle_points:
        if abs(distance(point,first) - r) < 0.001:
            # print(first, point)
            first = point
        if abs(distance(point,second) - r) < 0.001:
            second = point
        # if distance(point,first) - r < 0.001 or distance(point,first) - r > -0.001:
        #     # print(first, point)
        #     first = point
        # if abs(distance(point,second) - r) < 0.001 or distance(point,first) - r > -0.001:
            second = point
    return [first,*middle,second]

def get_neighbor_idx(idx, single_obstacle):
    if idx == 0:
        return len(single_obstacle) - 1, 1
    if idx == len(single_obstacle) - 1:
        return len(single_obstacle) - 2, 0
    else:
        return idx-1, idx+1
    
def heuristic_winding(Tether_config,obstacle,goal_pos,start_pos,goal_center_list, transit):
    # print("########################################################################################")
    transit_ = transit
    # print(Tether_config)
    goal_center1 = goal_center_list[0]
    goal_center2 =  goal_center_list[1]
    goal_direction = goal_center_list[2]

    start1 = start_pos[0][0:2]
    # print(start1)
    # start2 = start_pos[1][0:2]
    goal1 = goal_pos[0][0:2]
    # print(goal1)

    # goal2 = goal_pos[1][0:2]
    tether_config = cp.deepcopy(Tether_config)
    # print(tether_config)
    # print(tether_config)    
    original_tether_config = cp.deepcopy(Tether_config)
    # print(tether_config)
    robot1 = tether_config[0]
    start_robot = tether_config[0]
    # robot2 = tether_config[-1]
    # print(goal_center1, robot1)
    # print("##############")
    # print(robot1)
    cost = 0
    tag = True
    # print(robot1)
    # print(goal_center1, goal_center2)
    # print("###############")
    # print(robot1,goal_center1,goal_center2)
    if abs(distance(robot1,goal_center1)-1) > 0.01 and abs(distance(robot1,goal_center2)-1) > 0.01:
        # print("yes")
        # print(tether_config)
        if robot1 != start1:
            obstacle_class = []
            obstacle_points = []
            obstacle_polygon = []
            for i in obstacle:
                obstacle_class.append(Obstacle(i))
                obstacle_polygon.append(Polygon(i))
                for j in i:
                    obstacle_points.append(j) 
                    

            for idx1 in range(0,len(obstacle)):
                for idx2 in range(0,len(obstacle[idx1])): 
                    # print(obstacle[idx1][idx2])               
                    if abs(distance(obstacle[idx1][idx2],robot1) - 1) < 0.01:
                        obstacle_idx1 = idx1
                        vertex_idx1 = idx2
                        num_vertex1 = len(obstacle[idx1])
                        obstacle_1 = obstacle[idx1]
                        obstacle_1_class = Obstacle(obstacle_1)
                        angle_range1 = obstacle_class[idx1].angle[idx2]
            # print(obstacle_1)
            # print(vertex_idx1)
            # print(obstacle_idx1)
            # print(vertex_idx1)           
            neighbor_idx1, neighbor_idx2 = get_neighbor_idx(vertex_idx1, obstacle_1)
            base = obstacle_1[vertex_idx1]
            neighbor1 = obstacle_1[neighbor_idx1]
            neighbor2 = obstacle_1[neighbor_idx2]
            vector1 = np.array(neighbor1) - np.array(base)
            vector2 = np.array(neighbor2) - np.array(base)
            base_vector = np.array(tether_config[0])-np.array(tether_config[1])
            # print(vector1)
            # print(vector2)
            # print(base_vector)
            angle1 = np.arccos(np.dot(vector1, base_vector)/(np.linalg.norm(vector1)*np.linalg.norm(base_vector)))
            angle2 = np.arccos(np.dot(vector2, base_vector)/(np.linalg.norm(vector2)*np.linalg.norm(base_vector)))
            # print(angle1,angle2)
            # print(neighbor_idx1, neighbor_idx2)
            if angle1 < angle2:
                idx_order = [*list(range(vertex_idx1,-1,-1)),*list(range(num_vertex1 - 1, vertex_idx1, -1))]
            else:
                idx_order = [*list(range(vertex_idx1,num_vertex1)),*list(range(0,vertex_idx1))]
            # print(idx_order)
            # tag = 0

            for i in range(0,len(idx_order)):
                # print(i)
                # print("###############")
                # print(robot1)
                # print(obstacle_1)
                # print(obstacle_class[obstacle_idx1].angle[idx_order[i]])
                # print(goal1)
                # goal_check = get_goal(robot1,obstacle_1[idx_order[i]],obstacle_class[obstacle_idx1].angle[idx_order[i]], goal1)
                # print(robot1,obstacle_1[idx_order[i]], goal1, goal_direction)
                goal_check = get_goal_(robot1,obstacle_1[idx_order[i]],obstacle_class[obstacle_idx1].angle[idx_order[i]], goal1, goal_direction)
                # print(goal_check)
                # print(robot1,obstacle_1[idx_order[i]],obstacle_class[obstacle_idx1].angle[idx_order[i]], goal1, goal_direction)
                # print(goal_check)

                init = robot1
                # print(init)
                # print("###")
                # print(tether_config[0])
                # print(goal_check)
                # if goal_check != []:
                    # print(distance(goal_check[0][0], tether_config[0])/2)
                # if tag == 0 and goal_check != []:
                #     goal_check = []
                # print(distance(init, goal1))
                if goal_check != []:
                    # print("yes")
                    goal_check = goal_check[0]
                    # print(cost)
                    for poly in obstacle_polygon:
                        # print(poly)
                        if poly.intersects(LineString(goal_check)) ==True:
                            # print(original_tether_config,cost)
                            # print(distance(init, goal1))
                            cost = distance(start_robot, goal1)
                            return original_tether_config, cost
                    parent_tether_config = tether_config
                    # print(tether_config[0])
                    # print(goal_check[0])
                    tether_config1,length = tether_update_single(tether_config[0], goal_check[0], tether_config, obstacle, reverse = False)
                    robot1 = goal_check[0]
                                # print(robot1)
                                # print(distance(tether_config[0], goal_check[0])/2)
                                # print(np.arcsin(distance(tether_config[0], goal_check[0])/2))
                    # print(cost)
                    # cost = cost + np.arcsin(distance(tether_config[0], goal_check[0])/2)
                    # print(cost)
                                # print(tether_config[0])
                                # print(neighbor_list[0])
                                # print(distance(tether_config[0], neighbor_list[0]))
                                # print(0)
                                # print(distance(tether_config[0], goal_check[0])/2)
                                # print(np.arcsin(distance(tether_config[0], goal_check[0])/2))
                                # print(tether_config1[0])
                                # print(goal_check[1])
                                # print(tether_config1)
                    tether_config,length = tether_update_single(tether_config1[0], goal_check[1], tether_config1, obstacle, reverse = False)
                    robot1 = goal_check[1]
                    # print(tether_config)
                                # print(robot1)
                    # cost = cost + distance(tether_config1[0], goal_check[1])
                    # print(cost)
                                # print(1)
                                # print(cost)
                                        
                    if tether_not_cross_winding(tether_config1, parent_tether_config) == True and tether_not_cross_winding(tether_config, tether_config1) == True and tag == True:
                        # print(1)
                        # print(tether_config1, parent_tether_config)
                        # print(tether_config, tether_config1)
                        # print(transit_, robot1, goal_check)
                        vect0 = np.array(init) - np.array(transit_)
                        vect1 = np.array(goal_check[0]) - np.array(init)
                        vect2 = np.array(goal_check[1]) - np.array(goal_check[0])
                        if np.linalg.norm(vect1) != 0:
                            if np.linalg.norm(vect0) == 0:
                                vect0 = vect1
                            if np.linalg.norm(vect2) == 0:
                                vect2 = vect1                            
                            # print(distance(goal1, goal_check[1])/2)
                            if np.dot(vect0, vect1) < 0 or np.dot(vect1, vect2) < 0:
                                cost = distance(start_robot, goal1)
                                return original_tether_config, cost
                        
                        # cost = cost + np.arcsin(distance(goal1, goal_check[1])/2)
                        # print(cost)
                        cost = distance(start_robot, goal1)
                        # print(cost)
                        # print("valid")
                        return tether_config, cost
                    
                    else:
                        cost = distance(start_robot, goal1)
                        return original_tether_config, cost
                                    

                if goal_check == []:
                    # print("no")
                    # print(len(obstacle_1))
                    # print(idx_order[i])
                    # print(idx_order[i+1])
                    
                    # print(obstacle_1_class.angle)
                    # print(idx_order[i+1])
                    # print(obstacle_1_class.angle[idx_order[i+1]])
                    # neighbor_list = get_tangent_node(obstacle_1[idx_order[i]], obstacle_1[idx_order[i+1]], obstacle_1_class.angle[idx_order[i]], obstacle_1_class.angle[idx_order[i+1]])
                    try:
                        neighbor_list = get_tangent_node(obstacle_1[idx_order[i]], obstacle_1[idx_order[i+1]], obstacle_1_class.angle[idx_order[i]], obstacle_1_class.angle[idx_order[i+1]])
                    except:
                        neighbor_list = get_tangent_node(obstacle_1[idx_order[i]], obstacle_1[idx_order[0]], obstacle_1_class.angle[idx_order[i]], obstacle_1_class.angle[idx_order[0]])
                    # print(neighbor_list)
                    # print(neighbor_list,tether_config,11111)
                    neighbor_list = neighbor_list[0]
                    transit_ = neighbor_list[0]

                    parent_tether_config = tether_config
                        # print(neighbor_list)
                        # print(tether_config[0])
                        # print(neighbor_list[0])
                        # print(tether_config)
                        # print(tether_config[0], neighbor_list[0])
                    tether_config1,length = tether_update_single(tether_config[0], neighbor_list[0], tether_config, obstacle, reverse = False)
                        # print(tether_config1)
                    robot1 = neighbor_list[0]
                        # print(robot1)
                        # print(tether_config[0])
                        # print(neighbor_list[0])
                        # print(distance(tether_config[0], neighbor_list[0]))
                        # print(distance(tether_config[0], neighbor_list[0])/2)
                        # print(neighbor_list)
                        # print(tether_config)
                        # print(distance(tether_config[0], neighbor_list[0])/2)
                    #cost = cost + np.arcsin(distance(tether_config[0], neighbor_list[0])/2)
                    
                    tether_config,length = tether_update_single(tether_config1[0], neighbor_list[1], tether_config1, obstacle, reverse = False)
                    robot1 = neighbor_list[1]
                        # print(robot1)
                    #cost = cost + distance(tether_config1[0], neighbor_list[1])
                        # print(cost)
                        # print(robot1)
                    if tether_not_cross_winding(tether_config1, parent_tether_config) == False or tether_not_cross_winding(tether_config, tether_config1) == False:
                        tag = False
                        # cost = 0
                        # # print(2)
                        # # print(cost)
                        # # print(original_tether_config, cost)
                        # return original_tether_config, cost
                # tag = tag+1

    
        
    cost = distance(start_robot, goal1)
    # print(original_tether_config, cost)     
    return original_tether_config, cost

# obstacle = [[[6, 5], [12,5], [12,10], [6.1,10]],
#             [[15, 17.5], [20, 17.5], [19, 22], [15, 22]],
#             [[15.9,9.5],[18,6],[19.6,12],[17,13], [15.2, 12]],
#             [[2,17],[6,17.5],[6.7,21],[5,23], [1.9,19]]]
# # heuristic_winding(Tether_config,obstacle,goal_pos,start_pos,goal_center_list, transit)
# # tether_config = [[20.3596, 16.5669], [2, 17], [1.9, 19]]
# # tether_config = [[12.3043, 4.0474], [5.371, 4.2226],[4,10],[20,10]]
# tether_config = [[5.2029, 10.4419], [5.371, 4.2226]]
# # tether_config = [[5.0, 26.0], [1.9, 19], [2, 17], [6, 5], [12, 5], [15.2, 12], [20, 17.5], [19, 22], [15, 22], [6.6462, 16.2368]]
# # tether_config.reverse()

# goal_pos = [[5,26, np.pi/2],[18,26, np.pi/2]]
# # goal_pos.reverse()
# start_pos = [[0,0,0],[20,0,0]]
# # start_pos.reverse()
# goal_center_list = [[4,26],[6,26],np.pi/2]
# transit = [5.2029, 9.4419]
# tether_config, cost = heuristic_winding(tether_config,obstacle,goal_pos,start_pos,goal_center_list, transit)
# print(tether_config, cost)
# tether_config.reverse()
# goal_pos.reverse()
# goal_center_list = [[17,26],[19,26],np.pi/2]
# transit = [20.3596, 16.5669]

# print(heuristic_winding(tether_config,obstacle,goal_pos,start_pos,goal_center_list, transit))


            

                
    

 

        
                    
                            
        

    
    
    
def dubinsPath1(Parent_Robot_pair,goal_pos,start_pos,obstacle):
  
    start1 = start_pos[0]
    start2 = start_pos[1]
    goal1 = goal_pos[0]
    goal2 = goal_pos[1]
    tether_config = Parent_Robot_pair.tether_config
    
    parent_transit1 = Parent_Robot_pair.transit1
    # print("dubinspath1: parent transit 1")
    # print(parent_transit1)
    parent_transit2 = Parent_Robot_pair.transit2
    # print("dubinspath1: parent transit 2")
    # print(parent_transit2)
    
    
    goal_center1 = Parent_Robot_pair.tangent_node1.goal_center1
    goal_center2 =  Parent_Robot_pair.tangent_node1.goal_center2
    goal_direction1 = Parent_Robot_pair.tangent_node1.goal_direction
    
    cost_so_far = {}
    cost_so_far[list2tuple(tether_config)] = 0       
    tag = 0
    openlist = PriorityQueue()
    # Parent_Robot_pair.cost = 0
    openlist.put((0,tag,Parent_Robot_pair))
    
    while not openlist.empty():
        current_robot_pair = openlist.get()[2]
        robot1 = current_robot_pair.tangent_node1
        robot2 = current_robot_pair.tangent_node2
        tether_config = current_robot_pair.tether_config
        current_cost_to_go = current_robot_pair.cost
        
        if abs(distance(robot1.point, goal_center1) - 1) < 0.01 or abs(distance(robot1.point, goal_center2) - 1) < 0.01:
            
            new_tether_config, length = tether_update_single(robot1.point,goal1[0:2],tether_config, obstacle,False)
            new_tangent_node1 = tangent_node_([robot1.point, goal1[0:2]],robot1.point,goal1,start1, obstacle)
            new_tangent_node2 = tangent_node_([robot2.point, robot2.point],robot2.point,goal2,start2, obstacle)
            final_robot_pair = Robot_pair(new_tangent_node1,new_tangent_node2,new_tether_config,current_robot_pair)
            # final_robot_pair.transit1 = robot1.point
            final_robot_pair.transit1 = new_tether_config[0]
            # print(final_robot_pair.transit1)
            final_robot_pair.transit2 = parent_transit2
            final_robot_pair.cost = current_cost_to_go
            
            return final_robot_pair
        neighbor_list1 = robot1.neighbor
        for neighbor1 in neighbor_list1:
                if robot_direction_check(neighbor1, [tether_config[-1], tether_config[-1]], current_robot_pair.transit1, current_robot_pair.transit2, tether_config) == True:
            

                    new_cost = current_cost_to_go + g_cost(robot1,neighbor1[0], neighbor1[1])   
                    # print(tether_config)                             
                    # new_tether_config, length = tether_update([robot1.point, robot2.point], [neighbor1[1], robot2.point],tether_config, obstacle)                  
                    
                    new_tether_config1, length = tether_update_single(robot1.point,neighbor1[0],tether_config, obstacle,False)     
                    new_tether_config, length = tether_update_single(neighbor1[0],neighbor1[1],new_tether_config1, obstacle,False)                  
                    # print(new_tether_config)                          
                    # print(new_tether_config)              
                    if tether_not_cross_winding(new_tether_config, new_tether_config1) == True and tether_not_cross_winding(new_tether_config1, tether_config) == True:                        
                            if list2tuple(new_tether_config) not in cost_so_far or new_cost < cost_so_far[list2tuple(new_tether_config)]:
                                cost_so_far[list2tuple(new_tether_config)] = new_cost
                                # total_cost = new_cost + heuristic_cost(neighbor1[1], goal1)
                                total_cost = heuristic_cost(neighbor1[1], goal1)
                                new_tangent_node1 = tangent_node_(neighbor1,robot1.point,goal1,start1,obstacle)
                                # print([robot2.point,robot2.point])
                                new_tangent_node2 = tangent_node_([robot2.point,robot2.point],robot2.point,goal2,start2,obstacle)
                                New_robot_pair = Robot_pair(new_tangent_node1,new_tangent_node2,new_tether_config,current_robot_pair)
                                New_robot_pair.cost = new_cost
                                New_robot_pair.transit1 = neighbor1[0]
                                # print(New_robot_pair.transit1)
                                New_robot_pair.transit2 = new_tether_config[-1]
                                tag = tag+1
                                openlist.put((total_cost,tag,New_robot_pair))
    return False
# new_tether_config = [[5.0, 26.0], [1.9, 19], [2, 17], [6, 5], [12, 5], [12, 10], [2, 17], [1.9, 19], [5, 23], [15, 17.5], [20, 17.5], [19.9762, 22.2169]]
# tether_config = [[5.0, 26.0], [1.9, 19], [2, 17], [6, 5], [12, 5], [12, 10], [2, 17], [1.9, 19], [5, 23], [15, 17.5], [21.0, 16.5]]  
# print(tether_not_cross_winding(new_tether_config, tether_config))
def dubinsPath2(Parent_Robot_pair,goal_pos,start_pos,obstacle):
    start1 = start_pos[0]
    start2 = start_pos[1]
    goal1 = goal_pos[0]
    goal2 = goal_pos[1]
    tether_config = Parent_Robot_pair.tether_config
    # print(tether_config)    
    
    parent_transit1 = Parent_Robot_pair.transit1
    # print("dubins path2: parent_transit1")
    # print(parent_transit1)
    # print(Parent_Robot_pair.transit1)
    parent_transit2 = Parent_Robot_pair.transit2
    # print("dubins path2: parent_transit2")
    # print(parent_transit2)
    
    goal_center1 = Parent_Robot_pair.tangent_node2.goal_center1
    goal_center2 =  Parent_Robot_pair.tangent_node2.goal_center2
    goal_direction1 = Parent_Robot_pair.tangent_node2.goal_direction
 
    cost_so_far = {}
    cost_so_far[list2tuple(tether_config)] = 0       
    tag = 0
    openlist = PriorityQueue()
    # Parent_Robot_pair.cost = 0
    openlist.put((0,tag,Parent_Robot_pair))
    
    while not openlist.empty():
        current_robot_pair = openlist.get()[2]
        robot1 = current_robot_pair.tangent_node1
        robot2 = current_robot_pair.tangent_node2
        tether_config = current_robot_pair.tether_config
        current_cost_to_go = current_robot_pair.cost
        
        if abs(distance(robot2.point, goal_center1) - 1) < 0.01 or abs(distance(robot2.point, goal_center2) - 1) < 0.01:
            new_tether_config, length = tether_update_single(robot2.point,goal2[0:2],tether_config, obstacle,True)
            new_tangent_node1 = tangent_node_([robot1.point, robot1.point],robot1.point,goal1,start1, obstacle)
            new_tangent_node2 = tangent_node_([robot2.point, goal_pos[1][0:2]],robot2.point,goal2,start2, obstacle)
            final_robot_pair = Robot_pair(new_tangent_node1,new_tangent_node2,new_tether_config,current_robot_pair) 
            final_robot_pair.transit1 = new_tether_config[0]
            # print(final_robot_pair.transit1)
            # final_robot_pair.transit2 = robot2.point
            final_robot_pair.transit2 = new_tether_config[-1]  
            final_robot_pair.cost = current_cost_to_go       
            return final_robot_pair
        neighbor_list2 = robot2.neighbor
        for neighbor2 in neighbor_list2:
            # print(neighbor2)
            # print(parent_transit2)
            # print(tether_config)
            # print("########################")
            # print(tether_config)
            # print([tether_config[0], tether_config[0]], neighbor2, parent_transit1, parent_transit2, tether_config)
            # print(robot_direction_check( [tether_config[0], tether_config[0]], neighbor2, parent_transit1, parent_transit2, tether_config))
            # print(neighbor2)
            if robot_direction_check( [tether_config[0], tether_config[0]], neighbor2, current_robot_pair.transit1, current_robot_pair.transit2, tether_config) == True:
                new_cost = current_cost_to_go + g_cost(robot2,neighbor2[0], neighbor2[1])                             
                # new_tether_config, length = tether_update([robot1.point, robot2.point], [robot1.point, neighbor2[1]],tether_config, obstacle)                  
                new_tether_config1, length = tether_update_single(robot2.point, neighbor2[0],tether_config, obstacle,True) 
                new_tether_config, length = tether_update_single(neighbor2[0], neighbor2[1],new_tether_config1, obstacle,True) 
                # print(neighbor2)
                # print(new_tether_config)
                # print(tether_config)
                if tether_not_cross_winding(new_tether_config, new_tether_config1) == True and tether_not_cross_winding(new_tether_config1, tether_config) == True: 
                    
                        # print(neighbor2)
                        # print("1")                     
                        if list2tuple(new_tether_config) not in cost_so_far or new_cost < cost_so_far[list2tuple(new_tether_config)]:
                            # print(neighbor2)  
                            # print("2")  
                            cost_so_far[list2tuple(new_tether_config)] = new_cost
                            # total_cost = new_cost + heuristic_cost(neighbor2[1], goal2)
                            total_cost = heuristic_cost(neighbor2[1], goal2)
                            new_tangent_node1 = tangent_node_([robot1.point,robot1.point],robot1.point,goal1,start1,obstacle)
                            new_tangent_node2 = tangent_node_(neighbor2,robot2.point,goal2,start2,obstacle)
                            # print("check")
                            # print("################")
                            # print(neighbor2,robot2.point,goal2,start2,obstacle)
                            # print(new_tangent_node2.neighbor)
                            # print("################")
                            New_robot_pair = Robot_pair(new_tangent_node1,new_tangent_node2,new_tether_config,current_robot_pair)
                            New_robot_pair.cost = new_cost
                            New_robot_pair.transit1 = new_tether_config[0]
                            New_robot_pair.transit2 = neighbor2[0]
                            tag = tag+1
                            openlist.put((total_cost,tag,New_robot_pair))
    return False

# start_pos = [[0,0,np.pi/4],[20,0,np.pi/2]]
# tether_config = [start_pos[0][0:2],start_pos[1][0:2]]
# goal_pos = [[4,24],[22,26]]
# start1 = start_pos[0]
# start2 = start_pos[1]
# goal1 = goal_pos[0]
# goal2 = goal_pos[1]
# tangent_node1 = tangent_node(start1,None,goal1,start1)
# tangent_node2 = tangent_node(start2,None,goal2,start2)

# first_robot_pair = Robot_pair(tangent_node1, tangent_node2, tether_config, parent = None)
# first_robot_pair.cost = 0
# start_time = time.time() 
# current_robot_pair = dubinsPath1(first_robot_pair,goal_pos,start_pos)
# middle_time = time.time() - start_time
# print(middle_time)
# robot_pair = dubinsPath2(current_robot_pair,goal_pos,start_pos)
# print("--- %s seconds ---" % (time.time() - start_time)) 

# tether_config_list = []
# robot_pair_list = []
# while True:
#     tether_config_list.append(robot_pair.tether_config)
#     robot_pair_list.append(robot_pair)
#     robot_pair = robot_pair.parent
#     if robot_pair.parent == None:
#         tether_config_list.append(robot_pair.tether_config)
#         robot_pair_list.append(robot_pair)            
#         break
# tether_config_list = tether_config_list[::-1]
# print(tether_config_list)

def winding_constraint_check(winding_angle,winding_constraint):
    return max(winding_constraint - winding_angle,0)

# def reform(tether_config,obstacle_points):
#     # print(tether_config)
#     r = 1
#     first = tether_config[0]
#     second = tether_config[-1]
#     middle = tether_config[1:-1]
#     # print(first,second,middle)
#     for point in obstacle_points:
#         if abs(distance(point,first) - r) < 0.001:
#             # print(first, point)
#             first = point
#         if abs(distance(point,second) - r) < 0.001:
#             second = point
#         # if distance(point,first) - r < 0.001 or distance(point,first) - r > -0.001:
#         #     # print(first, point)
#         #     first = point
#         # if abs(distance(point,second) - r) < 0.001 or distance(point,first) - r > -0.001:
#             second = point
#     return [first,*middle,second]
    
def robot_direction_check(neighbor1, neighbor2, transit1, transit2, tether_config):
    # print("#################")
    # print(tether_config)
    # print(neighbor1[0])
    # print(neighbor2[0])
    if neighbor1 == [] or None:
        return False
    if neighbor2 == [] or None:
        return False
    if transit1 == None and transit2 == None:
        return True
    current_transit1 = transit1
    current_transit2 = transit2
    new_transit1 = neighbor1[0]
    new_transit2 = neighbor2[0]
    current_robot1 = tether_config[0]
    current_robot2 = tether_config[-1]
    new_neighbor1 = neighbor1[1]
    new_neighbor2 = neighbor2[1]
    vect1 = np.array(current_robot1) - np.array(current_transit1)
    vect1_ = np.array(new_transit1) - np.array(current_robot1)
    vect1__ = np.array(new_neighbor1) - np.array(new_transit1)
    vect2 = np.array(current_robot2) - np.array(current_transit2)
    vect2_ = np.array(new_transit2) - np.array(current_robot2)
    vect2__ = np.array(new_neighbor2) - np.array(new_transit2)
    # print("###########################")
    # print(vect1_)
    # print(vect2_)
    if np.linalg.norm(vect1) != 0:
        if np.linalg.norm(vect1_) == 0:
            vect1_ = vect1
        if np.linalg.norm(vect1__) == 0:
            vect1__ = vect1
        dot1 = np.dot(vect1, vect1_)/(np.linalg.norm(vect1)*np.linalg.norm(vect1_))
        dot1_ = np.dot(vect1_, vect1__)/(np.linalg.norm(vect1_)*np.linalg.norm(vect1__)) 
        if dot1 < 0 or dot1_ < 0:
            return False            

    if np.linalg.norm(vect2) != 0:
        if np.linalg.norm(vect2_) == 0:
            vect2_ = vect2
        if np.linalg.norm(vect2__) == 0:
            vect2__ = vect2
        dot2 = np.dot(vect2, vect2_)/(np.linalg.norm(vect2)*np.linalg.norm(vect2_))
        dot2_ = np.dot(vect2_, vect2__)/(np.linalg.norm(vect2_)*np.linalg.norm(vect2__)) 
        if dot2 < 0 or dot2_ < 0:
            return False 
        
    if np.linalg.norm(vect1) == 0:
        if np.linalg.norm(vect1_) != 0:
            if np.linalg.norm(vect1__) != 0:
                dot1_ = np.dot(vect1_, vect1__)/(np.linalg.norm(vect1_)*np.linalg.norm(vect1__)) 
                if dot1_ < 0:
                    return False  
                
    if np.linalg.norm(vect2) == 0:
        if np.linalg.norm(vect2_) != 0:
            if np.linalg.norm(vect2__) != 0:
                dot2_ = np.dot(vect2_, vect2__)/(np.linalg.norm(vect2_)*np.linalg.norm(vect2__)) 
                if dot2_ < 0:
                    return False 
        

    return True
# tether_config = [[5.0, 26.0], [1.9, 19], [2, 17], [6, 5], [12, 5], [12, 10], [2, 17], [1.9, 19], [5, 23], [15, 17.5], [20, 17.5], [19.9762, 22.2169]]
# print(robot_direction_check( [[5,26], [5,26]], [[19.866, 22.5], [18.134, 25.5]], [5,26], [20.9762, 17.7169], tether_config))
# # tether_config = [[19.9762, 22.2169], [15, 22], [15, 17], [19.6, 12], [18, 6], [6.7, 21], [5, 23], [1.9, 19], [2, 17], [5.0512999999999995, 4.6838]]
# # print(robot_direction_check(neighbor1, neighbor2, transit1, transit2, tether_config))    
# print(robot_direction_check( [tether_config[0], tether_config[0]], neighbor2, parent_transit1, parent_transit2, tether_config))
# [[0, 0], [20, 0]]
# [[18.2722, 5.0378], [15.030655776239989, 9.005792937509892]]
# [[20.566200000000002, 11.7423], [18, 6], [7.5585, 21.5129]]
# [[14.496, 16.1363], [19.6, 12], [18, 6], [5.7619, 23.6476]]
# [[14.0, 22.0], [15, 17], [19.6, 12], [18, 6], [6.7, 21], [5, 23], [1.1096, 19.6126]]
# [[19.0, 23.0], [15, 22], [15, 17], [19.6, 12], [18, 6], [6.7, 21], [5, 23], [1.9, 19], [1.0011999999999999, 16.9501]]        
# [[20.9762, 17.7169], [19, 22], [15, 22], [15, 17], [19.6, 12], [18, 6], [6.7, 21], [5, 23], [1.9, 19], [2, 17], [5.0512999999999995, 4.6838]]
# [[19.9762, 22.2169], [15, 22], [15, 17], [19.6, 12], [18, 6], [6.7, 21], [5, 23], [1.9, 19], [2, 17], [5.0512999999999995, 4.6838]]
# [[0.0, 26.0], [5, 23], [19.6, 12], [18, 6], [6.7, 21], [5, 23], [1.9, 19], [2, 17], [5.0512999999999995, 4.6838]]
# [[0.0, 26.0], [5, 23], [19.6, 12], [18, 6], [6.7, 21], [5, 23], [1.9, 19], [1.0513000000000003, 16.6838]]
# [[0.0, 26.0], [5, 23], [19.6, 12], [18, 6], [6.7, 21], [5, 23], [1.9, 19], [2, 17], [12.4315, 10.9021]]
# [[0.0, 26.0], [5, 23], [19.6, 12], [18, 6], [6.7, 21], [5, 23], [1.9, 19], [2, 17], [12, 10], [13.0, 5.0]]
# [[0.0, 26.0], [5, 23], [19.6, 12], [18, 6], [6.7, 21], [5, 23], [1.9, 19], [2, 17], [12, 10], [18.1644, 5.0136]]
# [[0.0, 26.0], [5, 23], [19.6, 12], [18, 6], [6.7, 21], [5, 23], [1.9, 19], [2, 17], [18, 6], [20.566200000000002, 11.7423]]  
# [[0.0, 26.0], [5, 23], [19.6, 12], [18, 6], [6.7, 21], [5, 23], [1.9, 19], [2, 17], [18, 6], [19.6, 12], [20.9974, 17.4275]] 
# [[0.0, 26.0], [5, 23], [19.6, 12], [18, 6], [6.7, 21], [5, 23], [1.9, 19], [2, 17], [18, 6], [19.6, 12], [20, 17.5], [19.9762, 22.2169]]
# [[0.0, 26.0], [5, 23], [19.6, 12], [18, 6], [6.7, 21], [5, 23], [1.9, 19], [2, 17], [18, 6], [19.6, 12], [20, 17.5], [19, 22], [18.0, 24.0]]   
    
        
    
# a = robot_direction_check([[5.0, 26.0],[5.0, 26.0]], [[14,17.5],[20,17.5]], [5.0, 26.0], [5.628, 23.7782], [[5.0, 26.0],[5.628, 23.7782],[14,17.5]])   
# print(a)
  
def winding_Astar(start_pos,goal_pos,tether_config,obstacle, winding_constraint):
    obstacle_points = []
    # obstacle_polygon = []
    for i in obstacle:
        # obstacle_polygon.append(Polygon(i))
        for j in i:
            obstacle_points.append(j)   
    start1 = start_pos[0]
    start2 = start_pos[1]
    goal1 = goal_pos[0]
    goal2 = goal_pos[1]
    tangent_node1 = tangent_node_(start1,None,goal1,start1,obstacle)
    tangent_node2 = tangent_node_(start2,None,goal2,start2,obstacle)
    
    start_center1 = tangent_node1.start_center1
    start_center2 = tangent_node1.start_center2
    goal_center1 = tangent_node1.goal_center1
    goal_center2 =  tangent_node1.goal_center2
    goal_direction1 = tangent_node1.goal_direction
    
    start_center3 = tangent_node2.start_center1
    start_center4 = tangent_node2.start_center2
    goal_center3 = tangent_node2.goal_center1
    goal_center4 =  tangent_node2.goal_center2
    goal_direction2 = tangent_node2.goal_direction
    # print(goal_center1,goal_center2,goal_center3,goal_center4)

    
    tag = 0
    iteration = 0
    openlist = PriorityQueue()
    close_list = {}
    close_list[list2tuple(tether_config)] = 0
    cost_so_far = {}
    first_robot_pair = Robot_pair(tangent_node1, tangent_node2, tether_config, parent = None)
    first_robot_pair.winding = 0
    first_robot_pair.cost = 0
    first_robot_pair.transit1 = start1[0:2]
    first_robot_pair.transit2 = start2[0:2]
    openlist.put((0,0,tag,first_robot_pair))
    
    while not openlist.empty():
        iteration += 1
        info = openlist.get()
        index = info[0]
        # print(index)
        current_robot_pair = info[3]
        robot1 = current_robot_pair.tangent_node1
        robot2 = current_robot_pair.tangent_node2
        tether_config = current_robot_pair.tether_config
        winding = current_robot_pair.winding
        current_cost_to_go = current_robot_pair.cost
        transit1 = current_robot_pair.transit1   
        transit2 = current_robot_pair.transit2 
        path_cost = current_robot_pair.cost
        print("######################")
        # print("current robot path cost")
        # print(path_cost)
        # print("current_cost_to_go")
        # print(index) 

        # # print(tether_config)
        # robot_pair1 = dubinsPath1(current_robot_pair,goal_pos,start_pos)
        # robot_pair = dubinsPath2(robot_pair1 ,goal_pos,start_pos) 
        # update_tether_config = robot_pair.tether_config
        # # if list2tuple(update_tether_config) not in close_list2:
        #     # close_list2[list2tuple(update_tether_config)] = 0
        # winding_angle = tether_winding_angle(update_tether_config)
        # # new_cost = winding_constraint_check(winding_angle,winding_constraint)   
        # robot_pair.winding = winding_angle    
        # # path_cost = robot_pair.cost
        goal_center_list = [goal_center1, goal_center2, goal_direction1]

        heuristic_tether1, cost1 = heuristic_winding(tether_config,obstacle,goal_pos,start_pos,goal_center_list, transit1) 
        goal_pos.reverse()
        start_pos.reverse()
        heuristic_tether1.reverse()
        # print(goal_pos, start_pos, heuristic_tether1)
        goal_center_list = [goal_center3, goal_center4, goal_direction2]
        heuristic_tether, cost2 = heuristic_winding(heuristic_tether1,obstacle,goal_pos,start_pos, goal_center_list, transit2)
        heuristic_tether.reverse()
        # print(new_tether_config)
        heuristic_winding_angle = tether_winding_angle(heuristic_tether)
        heuristic_winding_cost = winding_constraint_check(heuristic_winding_angle,winding_constraint)
        
        goal_pos.reverse()
        start_pos.reverse()
        heuristic_tether1.reverse() 
        
        total_path_cost = path_cost + cost1 + cost2     
        # print("total cost heuristics" )
        # print(total_path_cost)


        if heuristic_winding_angle >= winding_constraint:

            # print("final heuristic tether")
            # print(tether_config)
            # print(heuristic_tether)
            if abs(distance(heuristic_tether[0], goal_center1) - 1) < 0.01 or  abs(distance(heuristic_tether[0], goal_center2)-1) < 0.01:
                    if abs(distance(heuristic_tether[-1], goal_center3) - 1) < 0.01 or  abs(distance(heuristic_tether[-1], goal_center4) - 1) < 0.01:
                        robot_pair1 = dubinsPath1(current_robot_pair,goal_pos,start_pos,obstacle)
                        robot_pair = dubinsPath2(robot_pair1 ,goal_pos,start_pos,obstacle) 
                        update_tether_config = robot_pair.tether_config
                        # if list2tuple(update_tether_config) not in close_list2:
                            # close_list2[list2tuple(update_tether_config)] = 0
                        winding_angle = tether_winding_angle(update_tether_config) 
                        robot_pair.winding = winding_angle    
                        path_cost = robot_pair.cost  
                        print(heuristic_winding_angle)
                        print(path_cost)   
                        print(iteration)
                        return robot_pair
        # print("############################################################################################")

        # print("#####################")       
        # if index == -1:   
        #     # return current_robot_pair   
        #     robot_pair1 = dubinsPath1(current_robot_pair,goal_pos,start_pos,obstacle)
        #     robot_pair = dubinsPath2(robot_pair1 ,goal_pos,start_pos,obstacle) 
        #     update_tether_config = robot_pair.tether_config
        #     # if list2tuple(update_tether_config) not in close_list2:
        #         # close_list2[list2tuple(update_tether_config)] = 0
        #     winding_angle = tether_winding_angle(update_tether_config) 
        #     robot_pair.winding = winding_angle    
        #     path_cost = robot_pair.cost  
        #     print("###########################")
        #     # print(robot_pair.tether_config)
        #     print(path_cost)   
        #     return robot_pair
            # print("########################")
            # print("########################")
               
        # if winding_constraint_check(winding,winding_constraint) < 0.01:
        #     return current_robot_pair
        
        neighbor_list1 = robot1.neighbor
        # print("current position")
        # print(robot1.point)
        # print("neighbor1:")
        # print(neighbor_list1)
        neighbor_list2 = robot2.neighbor
        for neighbor1 in neighbor_list1:
            # print("neighbor1")
            # print(neighbor1[1]) 
            # time.sleep(0.2)
            for neighbor2 in neighbor_list2: 
                # print("neighbor2")
                # print(neighbor2[1]) 
                # time.sleep(0.2)
                # print(robot_direction_check(neighbor1, neighbor2, current_robot_pair.transit1, current_robot_pair.transit2, tether_config))
                if neighbor1 != [] and neighbor2 != []:#
                    # print("##############")
                    # print("neighbor1")
                    # print(neighbor1) 
                    # print("neighbor2")
                    # print(neighbor2) 
                    # print("tether_config")
                    # print(tether_config)
                    # print("transit1")
                    # print(current_robot_pair.transit1)
                    # print("transit2")
                    # print(current_robot_pair.transit2)
                    
                    if robot_direction_check(neighbor1, neighbor2, current_robot_pair.transit1, current_robot_pair.transit2, tether_config) == True:
                        # print("neighbor2")
                        # print(neighbor2) 
                        # print("##############")
                        # print(robot1.point)
                        # print(robot2.point)
                        # print(neighbor1[0])
                        # print(neighbor2[0])                    
                        # print(neighbor1[1])
                        # print(neighbor2[1])
                        # print(g_cost(robot1,neighbor1[0], neighbor1[1]))
                        # print(g_cost(robot2,neighbor2[0], neighbor2[1]) )
                        # print(current_cost_to_go)
                        path_cost = current_cost_to_go + g_cost(robot1,neighbor1[0], neighbor1[1]) + g_cost(robot2,neighbor2[0], neighbor2[1])                    
                        new_tether_config1, length1 = tether_update([robot1.point, robot2.point], [neighbor1[0], neighbor2[0]],tether_config, obstacle)  
                        new_tether_config, length = tether_update([neighbor1[0], neighbor2[0]], [neighbor1[1], neighbor2[1]],new_tether_config1, obstacle)                
                        # print("new tether_config")
                        # print(new_tether_config)
                        # print("new tether_config")
                        # print(new_tether_config)
                        # print("#################")
                        # time.sleep(0.2)
                        new_tether_config_reform = reform(new_tether_config,obstacle_points)
                        # print(new_tether_config_reform)
                        new_winding = tether_winding_angle(new_tether_config) 
                        # print("new tether_config")
                        # print(new_tether_config)

                        # if current_robot_pair.parent != None:
                        #     grandparent = current_robot_pair.parent.tether_config 
                        #     # if distance(new_tether_config[0], grandparent[0]) > 2 and distance(new_tether_config[-1], grandparent[-1]) > 2: 
                        #     #     print("##############")
                        #     #     print(grandparent)
                        #     #     print(tether_config)
                        #     #     print(new_tether_config)
                        #     # grandparent_form = reform(grandparent,obstacle_points) 
                        #     if distance(new_tether_config[0], grandparent[0]) < 2 or distance(new_tether_config[-1], grandparent[-1]) < 2:             
                        #     # if list2tuple(new_tether_config) == list2tuple(grandparent):
                        #         print(0)
                        #         continue    

                        # print("new tether_config4")
                        # print(new_tether_config)
                        if tether_not_cross(new_tether_config) == True:                            
                        # if tether_not_cross_winding(new_tether_config, tether_config) == True: 
                            if len(new_tether_config) >= 2:
                                    # print(new_tether_config)
                                    # time.sleep(0.2)
                                # if new_winding >= winding:
                                    if list2tuple(new_tether_config_reform) not in close_list or path_cost < close_list[list2tuple(new_tether_config_reform)]:    
                                            close_list[list2tuple(new_tether_config_reform)] = path_cost       
                                            new_tangent_node1 = tangent_node_(neighbor1,robot1.point,goal1,start1,obstacle)
                                            new_tangent_node2 = tangent_node_(neighbor2,robot2.point,goal2,start2,obstacle)
                                            New_robot_pair = Robot_pair(new_tangent_node1,new_tangent_node2,new_tether_config,current_robot_pair)                            
                                            New_robot_pair.winding =  new_winding
                                            New_robot_pair.cost = path_cost
                                            New_robot_pair.transit1 = neighbor1[0]
                                            New_robot_pair.transit2 = neighbor2[0]
                                            new_cost0 = winding_constraint_check(new_winding,winding_constraint) 
                                            # print("neighbor2")
                                            # print(neighbor2) 
                                            # print("tether config")
                                            # print("################")
                                            print(new_tether_config)
                                            # print(current_cost_to_go < path_cost)
                                            # print(path_cost)
                                            goal_center_list = [goal_center1, goal_center2, goal_direction1]

                                            heuristic_tether1, cost1 = heuristic_winding(new_tether_config,obstacle,goal_pos,start_pos,goal_center_list, neighbor1[0]) 
                                            goal_pos.reverse()
                                            start_pos.reverse()
                                            heuristic_tether1.reverse()
                                            # print(goal_pos, start_pos, heuristic_tether1)
                                            goal_center_list = [goal_center3, goal_center4, goal_direction2]
                                            heuristic_tether, cost2 = heuristic_winding(heuristic_tether1,obstacle,goal_pos,start_pos, goal_center_list, neighbor2[0])
                                            heuristic_tether.reverse()
                                            # print(new_tether_config)
                                            heuristic_winding_angle = tether_winding_angle(heuristic_tether)
                                            heuristic_winding_cost = winding_constraint_check(heuristic_winding_angle,winding_constraint)
                                            
                                            goal_pos.reverse()
                                            start_pos.reverse()
                                            heuristic_tether1.reverse() 
                                            # print(heuristic_tether)
                                            # print(heuristic_tether)
                                            # print(cost1,cost2)
                                            # print(path_cost)
                                            # print(cost1)
                                            # print(cost2)
                                
                                            total_path_cost = path_cost + cost1 + cost2
                                            # print(total_path_cost) 
                                            # print("-----------------------------")  
                                            # print(index)
                                            # print(total_path_cost)
                                            # if current_cost_to_go > total_path_cost:
                                            #     print("path cost")
                                            #     print(path_cost) 
                                            #     print("total path cost")
                                            #     print(total_path_cost)
                                            #     print("cost1")
                                            #     print(cost1)
                                            #     print("cost2")
                                            #     print(cost2)


                                            # if heuristic_winding_angle >= winding_constraint:
                                            #     if abs(distance(heuristic_tether[0], goal_center1) - 1) < 0.01 or  abs(distance(heuristic_tether[0], goal_center2)-1) < 0.01:
                                            #             if abs(distance(heuristic_tether[-1], goal_center3) - 1) < 0.01 or  abs(distance(heuristic_tether[-1], goal_center4) - 1) < 0.01:
                                            #                 tag = tag + 1
                                            #                 openlist.put((-1,total_path_cost,tag,New_robot_pair))
                                            #                 print("heuristic winding angle")
                                            #                 print(heuristic_winding_angle)
                                            #                 # print(New_robot_pair.transit1) 
                                            #                 # print(total_path_cost)
                                            #                 # # print(new_tether_config)
                                            #                 # print(heuristic_tether)
                                            #                 # print(total_path_cost)

                                            tag = tag+1
                                            gamma = 0.9
                                            # print(New_robot_pair.tether_config)
                                            if index >= total_path_cost:
                                                print("big error")
                                            if index <= total_path_cost:
                                                openlist.put((gamma*heuristic_winding_cost+(1-gamma)*total_path_cost,path_cost,tag,New_robot_pair)) 
                                            # openlist.put((new_cost0+0.1*New_robot_pair.cost,path_cost,tag,New_robot_pair))                                           
                                            
    return False

def is_the_input_valid(start_pos,goal_pos,obstacle,length_limit,r):
    obstacle_points = []
    obstacle_polygon = []
    for i in obstacle:
        obstacle_polygon.append(Polygon(i))
        for j in i:
            obstacle_points.append(j)
    start1 = start_pos[0][0:2]
    start2 = start_pos[1][0:2]  
    goal1 = goal_pos[0][0:2]
    goal2 = goal_pos[1][0:2]
    if start_pos[0][2] < 0 or start_pos[0][2] > np.pi:
        return False
    if start_pos[1][2] < 0 or start_pos[1][2] > np.pi:
        return False
    if goal_pos[0][2] < 0 or goal_pos[0][2] > np.pi:
        return False
    if goal_pos[0][2] < 0 or goal_pos[0][2] > np.pi:
        return False
    check_list = [start1,start2,goal1,goal2]
    for point in check_list:
        p = Point(point)
        for obs in obstacle_points:
            if distance(point, obs) < r:
                return False
        for poly in obstacle_polygon:
            if poly.intersects(p) == True:
                return False
    if distance(start1,start2) > length_limit or distance(goal1,goal2) > length_limit:
        return False
    theta1 = start_pos[0][-1]
    theta2 = start_pos[1][-1]
    vect1 = [r*np.cos(theta1+np.pi/2),r*np.sin(theta1+np.pi/2)]
    c1 = np.array(start1) + np.array(vect1)        
    vect2 = [r*np.cos(theta2+np.pi/2),r*np.sin(theta2+np.pi/2)]
    c2 = np.array(start2) + np.array(vect2)
    
    if distance(c1,goal1) < r or distance(c2,goal2) < r:
        return False
    
    return True

