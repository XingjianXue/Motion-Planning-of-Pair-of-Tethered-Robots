#
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
from algorithm import tether_hybrid_Astar1,tether_hybrid_Astar2,is_the_input_valid,Robot_pair,Robot, g_cost,heuristic_cost,list2tuple
from plot import plot



def main():     
    for s in range(0,4):
        print("##################################################")      
        start_pos = [[5,0,np.pi/4],[15,0,np.pi/2]]
        r = 1
        step_n = 0.3
        # goal_pos = [[0,5],[15,-5]]
        
        goal_pos = [[4,24],[18,24]]
        # goal_pos = [[0,10],[4,24]]        
        # tether_config = [[0,0],[18,0]]
        tether_config = [start_pos[0][0:2],start_pos[1][0:2]]
        length_limit = 100
        # length_limit = 40      
        
        input_check = is_the_input_valid(start_pos,goal_pos,obstacle,length_limit,r)
        if input_check == False:
            print("The input is invalid")
            return 0
        else: 
            print("valid input")
            
        # plt.show()
        cost_sum = 0
        start_time = time.time()    
        robot_pair_m = tether_hybrid_Astar1(start_pos,goal_pos,tether_config,length_limit, obstacle)

        if robot_pair_m == False:
            print("no solution found")
            return 0
        
        cost_sum = cost_sum + robot_pair_m.cost
        

        
        robot_pair,index = tether_hybrid_Astar2(goal_pos,robot_pair_m,length_limit, obstacle,start_pos)
        print("--- %s seconds ---" % (time.time() - start_time)) 
        time_count = time.time() - start_time
        if robot_pair_m == False:
            print("no solution found")
            return 0
        cost_sum = cost_sum + robot_pair.cost
        print("total cost")
        print(cost_sum)
        tether_config_list = []
        robot_pair_list = []
        
        # while True:
        #     print("#########################")
        #     print(robot_pair_m.tangent_node1.point)
        #     print(robot_pair_m.tangent_node2.point)   
        #     print(robot_pair_m.tether_config)
        #     print(robot_pair_m.num)  
        #     robot_pair_m = robot_pair_m.parent

        #     if robot_pair_m.parent == None:      
        #         print(robot_pair_m.tangent_node1.point)
        #         print(robot_pair_m.tangent_node2.point)   
        #         print(robot_pair_m.tether_config)  
        #         print(robot_pair_m.num)
        #         break

        while True:
            tether_config_list.append(robot_pair.tether_config)
            robot_pair_list.append(robot_pair)
            robot_pair = robot_pair.parent
            if robot_pair.parent == None:
                tether_config_list.append(robot_pair.tether_config)
                robot_pair_list.append(robot_pair)            
                break
        tether_config_list = tether_config_list[::-1]
        # print(tether_config_list)
        robot_pair_list = robot_pair_list[::-1]
        
        plot(start_pos,goal_pos,robot_pair_list,cost_sum,length_limit,index,tether_config_list,time_count,step_n)    
   
if __name__ == '__main__':
    main()   
                        
        
    
    
    
    
    
    
    