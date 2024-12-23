import numpy as np
from obstacle_class import Obstacle
from tether_update import distance, tether_update,tether_update_single,tether_not_cross,tether_winding_angle
from shapely.geometry import Polygon, LineString, Point
from queue import PriorityQueue 
from tangent_node import tangent_node,get_tangent_node,get_goal,get_goal_from_start,obstacle,distance_sum,rearrange
import copy as cp
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as P
from matplotlib.patches import Circle
from shapely import is_simple
import time
# from algorithm import Robot_pair,Robot, g_cost,heuristic_cost,list2tuple,tether_hybrid_Astar1,tether_hybrid_Astar2,is_the_input_valid

def plot(start_pos,goal_pos,robot_pair_list,cost_sum,length_limit,index,tether_config_list,time_count,step_n):
    r = 1
    obstacle_points = []
    for i in obstacle:
        for j in i:
            obstacle_points.append(j)       
    fig,ax = plt.subplots()
    ax.set_xlim([-5,30])
    ax.set_ylim([-5,30])
    for i in obstacle:
        p = P(i, facecolor = 'k')
        ax.add_patch(p)
    for point in obstacle_points:
        circle = Circle((point[0],point[1]), 1,color = "k", fill = False, linestyle = "--")
        ax.add_patch(circle)        
    start1 = Circle((start_pos[0][0], start_pos[0][1]), 0.5, color = 'r', fill = True)
    start2 = Circle((start_pos[1][0], start_pos[1][1]), 0.5, color = 'r', fill = True)
    goal1 = Circle((goal_pos[0][0], goal_pos[0][1]), 0.5, color = 'b', fill = True)
    goal2 = Circle((goal_pos[1][0], goal_pos[1][1]), 0.5, color = 'b', fill = True)
    ax.add_patch(start1)
    ax.add_patch(start2)
    ax.add_patch(goal1)
    ax.add_patch(goal2)
    first_robot = robot_pair_list[0]
    center1 = first_robot.tangent_node1.center
    center2 = first_robot.tangent_node2.center
    start_circle1 = Circle((center1[0], center1[1]), r, color = "k", fill = False, linestyle = "--")    
    start_circle2 = Circle((center2[0], center2[1]), r, color = "k", fill = False, linestyle = "--") 
    ax.add_patch(start_circle1)
    ax.add_patch(start_circle2) 
    plt.pause(10)
    name = "Cost:"+ str(np.round(cost_sum,3))+ "  L:"+str(np.round(length_limit,3))+"  Time:"+ str(np.round(time_count,3))
    plt.title(name)
    # i = 0
    # for tether_config in tether_config_list:
    #     i = i + 1
    #     tether_plot = ax.plot([i[0] for i in tether_config],[i[1] for i in tether_config],'r')
    #     plt.pause(1)
    #     if i != len(tether_config_list):
    #         line = tether_plot.pop(0)
    #         line.remove()
            

    # print("#####################")        
    for i in range(1,len(robot_pair_list)):
        tag = i
        robot_pair = robot_pair_list[i]
        # print(robot_pair.num)
        
        if robot_pair.num == 2:
            # print('2 robot moving')

            tether_config = robot_pair.tether_config 
            # print(tether_config)     
            robot1 = robot_pair.tangent_node1       
            pos1 = robot1.point
            parent1 = robot1.parent
            parent_center1 = robot1.parent_center
            parent_angle_range1 = robot1.parent_angle_range
            transit1 = robot1.transit
            vect1 = np.array(transit1)-np.array(parent_center1)     
            transit1_angle = np.arctan2(vect1[1],vect1[0])
            vect1_ = np.array(parent1)-np.array(parent_center1)
            parent1_angle = np.arctan2(vect1_[1],vect1_[0])
            
            robot2 = robot_pair.tangent_node2
            pos2 = robot2.point
            parent2 = robot2.parent
            parent_center2 = robot2.parent_center
            parent_angle_range2 = robot2.parent_angle_range
            transit2 = robot2.transit
            vect2 = np.array(transit2)-np.array(parent_center2)
            transit2_angle = np.arctan2(vect2[1], vect2[0])
            vect2_ = np.array(parent2)-np.array(parent_center2)
            parent2_angle = np.arctan2(vect2_[1],vect2_[0])
                
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
            # print(tether_config)
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


            
            # trajectory1 = [] 
            # trajectory2 = []
                
                                                                            
        if robot_pair.num == 1:
            tether_config = robot_pair.tether_config 
            # print(tether_config)   
            robot1 = robot_pair.tangent_node       
            pos1 = robot1.point
            parent1 = robot1.parent
            parent_center1 = robot1.parent_center
            parent_angle_range1 = robot1.parent_angle_range
            transit1 = robot1.transit
            vect1 = np.array(transit1)-np.array(parent_center1)     
            transit1_angle = np.arctan2(vect1[1],vect1[0])
            vect1_ = np.array(parent1)-np.array(parent_center1)
            parent1_angle = np.arctan2(vect1_[1],vect1_[0])
            # print("##########")
            # print(pos1)
            # print(transit1)
            # print(parent1)
            # print(parent_center1)
            # print(parent1_angle)
            # print(transit1_angle)
            # print("##################")
            # print(robot_pair.tangent_node.point)
            # print(robot_pair.tangent_node.center)
            # print(robot_pair.tangent_node.neighbor)
            # print("##################")
            
            
            angle1_range = []
            line1_range = []

            if max(transit1_angle,parent1_angle) - min(transit1_angle,parent1_angle) < np.pi:
                # print("a")
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
                # print(trajectory1)
                arc_pos1 = []
                line1_range = []            
                
            if max(transit1_angle,parent1_angle) - min(transit1_angle,parent1_angle) > np.pi:    
                if transit1_angle >= parent1_angle:
                    # print("b")
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
                    # print(trajectory1)
                    arc_pos1 = []
                    line1_range = []  
                    parent1_angle = parent1_angle - 2*np.pi    
                                    
                if transit1_angle < parent1_angle:
                    # print("c")
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
    
            if index == 0:
                tether_config = tether_config_list[tag - 1]
                for i in range(0,len(trajectory1)-1):
                    tether_config, length = tether_update([trajectory1[i], goal_pos[1]], [trajectory1[i+1], goal_pos[1]], tether_config, obstacle)
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
                trajectory1 = []
        
            if index == 1: 
                tether_config = tether_config_list[tag - 1]   
                for i in range(0,len(trajectory1)-1):
                    tether_config, length = tether_update([goal_pos[0], trajectory1[i]], [goal_pos[0], trajectory1[i+1]], tether_config, obstacle)
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
                trajectory1 = [] 
                
    final_tether = tether_config_list[-1]
    tether_plot = ax.plot([i[0] for i in final_tether],[i[1] for i in final_tether],'r')  
    # name = "Cost:"+ str(np.round(cost_sum,3))+ "  L:"+str(np.round(length_limit,3))+"  Time:"+ str(np.round(time_count,3))
    # plt.title(name)
    
                
                
        
        

    # for i in range(0,len(tether_config_list) - 1):
    #     n = 30
    #     tether_config = tether_config_list[i]
    #     start = tether_config_list[i]

    #     goal = tether_config_list[i+1]

    #     trajectory1 = np.array(goal[0]) - np.array(start[0])

    #     trajectory2 = np.array(goal[-1]) - np.array(start[-1])
    #     for j in range(0,n):
    #         j = j+1
    #         goal1 = np.array(start[0])+trajectory1*j/n
    #         goal2 = np.array(start[-1])+trajectory2*j/n
    #         start1 = goal1 - trajectory1/n
    #         start2 = goal2 - trajectory2/n
    #         tether_config, length = tether_update([start1, start2], [list(goal1), list(goal2)], tether_config, obstacle)
    #         tether_plot = ax.plot([i[0] for i in tether_config],[i[1] for i in tether_config],'r')      
    #         plt.pause(0.05)  
    #         line = tether_plot.pop(0)
    #         line.remove()
    name_save = "Cost"+ str(np.round(cost_sum,3))+ "L"+str(np.round(length_limit,3))+"Time"+ str(np.round(time_count,3))+".jpg"
    # print(name_save)
    plt.savefig(name_save)                 
    # plt.show()