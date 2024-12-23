import numpy as np
from shapely.geometry import Polygon, LineString, Point
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon as P
import copy as cp
from shapely import is_simple, intersection, get_coordinates


# obstacle = [[[6, 5], [12,5], [12,10], [6.1,10]],
#             [[15, 17], [20, 17.5], [19, 22], [15, 22]],
#             [[15.9,9.5],[18,6],[19.6,12],[17,13], [15.2, 12]],
#             [[2,17],[6,17],[6.7,21],[5,23], [1.9,19]]]
# start_pos = [[0,0], [25,0]]
# end_pos = [[0,35],[25,35]]
# Tether_config = [[0,0], [25,0]]
# start_tether = cp.deepcopy(Tether_config)

# start_pos = [[0.0, 35.0], [25.0, 35.0]]
# end_pos = [[0,0], [25,0]]
# Tether_config = [[0.0, 35.0], [2, 17], [6, 5], [12, 5], [18, 6], [19.6, 12], [25.0, 35.0]]
# start_tether = cp.deepcopy(Tether_config)

# start_pos = [[15, 0], [10, 30]]
# end_pos = [[0,0], [-5, 20]]
# Tether_config = [[15, 0], [10, 30]]
# start_tether = cp.deepcopy(Tether_config)

# start_pos = [15, 0]
# end_pos = [0,0]
# Tether_config = [[15, 0], [10, 30]]
# start_tether = cp.deepcopy(Tether_config)

# tether_config = [[5.7619393177594596-0.01, 23.64764842009554], [6.7, 21], [6.1, 10], [6, 5], [12, 5], [12, 10], [6.985030467155704, 16.827619668247753]]

# start_pos = [tether_config[0],tether_config[-1]]
# end_pos = [[1.1095849478247564, 19.612571665435816],[15.0, 16.0]]


def distance(point1,point2):
    return np.sqrt((point1[0] - point2[0])**2 + (point1[1] - point2[1])**2)    
        
def tether_update(start_pos, end_pos, tether_config, obstacle):
    Tether_config = cp.deepcopy(tether_config)
    obstacle_points = []
    obstacle_polygon = []
    for i in obstacle:
        obstacle_polygon.append(Polygon(i))
        for j in i:
            obstacle_points.append(j)

        

    vect = np.array(end_pos) - np.array(start_pos)
    iter = 25

    for i in range(iter):
        i = i+1
        if len(Tether_config) == 2:
            anchor1 = Tether_config[0]
            anchor2 = Tether_config[1]
            Tether_config.pop(0)   
            pos1 = start_pos[0] + i/iter*vect[0,:]
            pos2 = pos1 -1/iter*vect[0,:]
            for j in obstacle_points:
                poly = Polygon([pos1,pos2,anchor2])
                vert = Point(j)
                if poly.intersects(vert) == True and j != anchor1 and j != anchor2:
                    Tether_config.insert(0,j)
            Tether_config.insert(0,list(pos1))
            # print(Tether_config)
                    
        if len(Tether_config) > 2:
            anchor1 = Tether_config[0]
            anchor2 = Tether_config[1]
            anchor3 = Tether_config[2]
            Tether_config.pop(0)
            pos1 = start_pos[0] + i/iter*vect[0,:]
            pos2 = pos1 - 1/iter*vect[0,:]
            
            area_intersect = 0
            poly2 = Polygon([pos1,anchor2,anchor3])
            for i in obstacle_polygon:
                area_intersect = area_intersect + i.intersection(poly2).area
            if area_intersect == 0:
                Tether_config.pop(0)
                
                    
            for j in obstacle_points:
                # print(pos1,pos2,anchor2)
                poly = Polygon([pos1,pos2,anchor2])
                vert = Point(j)
                if poly.intersects(vert) == True and j != anchor1 and j != anchor2:
                    Tether_config.insert(0,j)
            Tether_config.insert(0,list(pos1))
            # print(Tether_config)

    Tether_config.reverse()
    # print(Tether_config)

    for i in range(iter):
        i = i+1
        if len(Tether_config) == 2:
            anchor1 = Tether_config[0]
            anchor2 = Tether_config[1]
            Tether_config.pop(0)   
            pos1 = start_pos[1] + i/iter*vect[1,:]
            pos2 = pos1 -1/iter*vect[1,:]
            for j in obstacle_points:
                poly = Polygon([pos1,pos2,anchor2])
                vert = Point(j)
                if poly.intersects(vert) == True and j != anchor1 and j != anchor2:
                    Tether_config.insert(0,j)
            Tether_config.insert(0,list(pos1))
            # print(Tether_config)
            
                    
        if len(Tether_config) > 2:
            anchor1 = Tether_config[0]
            anchor2 = Tether_config[1]
            anchor3 = Tether_config[2]
            Tether_config.pop(0)
            pos1 = start_pos[1] + i/iter*vect[1,:]
            pos2 = pos1 - 1/iter*vect[1,:]
            
            area_intersect = 0
            poly2 = Polygon([pos1,anchor2,anchor3])
            for i in obstacle_polygon:
                area_intersect = area_intersect + i.intersection(poly2).area
            if area_intersect == 0:
                Tether_config.pop(0)
                
                    
            for j in obstacle_points:
                poly = Polygon([pos1,pos2,anchor2])
                vert = Point(j)
                if poly.intersects(vert) == True and j != anchor1 and j != anchor2:
                    Tether_config.insert(0,j)
            Tether_config.insert(0,list(pos1))
            # print(Tether_config)
    Tether_config.reverse()
    length = 0
    for i in range(len(Tether_config) - 1):
        length = length + distance(Tether_config[i],Tether_config[i+1])
    # print(Tether_config)  
    return Tether_config, length

def tether_update_single(start_pos, end_pos, tether_config, obstacle, reverse):
    
        
    Tether_config = cp.deepcopy(tether_config)
    if reverse == True:
       Tether_config.reverse() 
    obstacle_points = []
    obstacle_polygon = []
    for i in obstacle:
        obstacle_polygon.append(Polygon(i))
        for j in i:
            obstacle_points.append(j)

        

    vect = np.array(end_pos) - np.array(start_pos)
    iter = 25

    for i in range(iter):
        i = i+1
        if len(Tether_config) == 2:
            anchor1 = Tether_config[0]
            anchor2 = Tether_config[1]
            # print(Tether_config)
            Tether_config.pop(0)   
            pos1 = start_pos + i/iter*vect
            pos2 = pos1 -1/iter*vect
            for j in obstacle_points:
                poly = Polygon([pos1,pos2,anchor2])
                vert = Point(j)
                if poly.intersects(vert) == True and j != anchor1 and j != anchor2:
                    Tether_config.insert(0,j)
            Tether_config.insert(0,list(pos1))
            # print(Tether_config)
                    
        if len(Tether_config) > 2:
            anchor1 = Tether_config[0]
            anchor2 = Tether_config[1]
            anchor3 = Tether_config[2]
            Tether_config.pop(0)
            pos1 = start_pos + i/iter*vect
            pos2 = pos1 - 1/iter*vect
            
            area_intersect = 0
            poly2 = Polygon([pos1,anchor2,anchor3])
            for i in obstacle_polygon:
                area_intersect = area_intersect + i.intersection(poly2).area
            if area_intersect == 0:
                Tether_config.pop(0)
                
                    
            for j in obstacle_points:
                poly = Polygon([pos1,pos2,anchor2])
                vert = Point(j)
                if poly.intersects(vert) == True and j != anchor1 and j != anchor2:
                    Tether_config.insert(0,j)
            Tether_config.insert(0,list(pos1))
            # print(Tether_config)
    if reverse == True:
        Tether_config.reverse()
    length = 0
    for i in range(0,len(Tether_config) - 1):
        length = length + distance(Tether_config[i],Tether_config[i+1])
    # print(Tether_config)  
    return Tether_config, length

# print(tether_update_single([5.146149062397057, 4.479482395730391], [3, 8], [[5.146149062397057, 4.479482395730391], [18, 5]], obstacle, reverse = False))

def compare(line_intersect, tether_config):
    tag = 0
    for point in tether_config:
        if point == line_intersect[0]:
            tag = tag + 1
    if tag == 0:
        return False
  
def tether_config_extend(tether_config):
    vect1 = np.array(tether_config[0]) - np.array(tether_config[1])
    vect2 = np.array(tether_config[-1]) - np.array(tether_config[-2])
    new_start_point = vect1*100 + tether_config[0]
    new_end_point = vect2*100 + tether_config[-1]
    # print(new_start_point, new_end_point)
    tether_config_ = cp.deepcopy(tether_config)
    # print(tether_config)
    tether_config_[0] = list(new_start_point)
    tether_config_[-1] = list(new_end_point)
    return tether_config_

# print(tether_config_extend([[0,1],[1,1],[1,2]]))
def tether_not_cross_winding(tether_config, parent_tether_config):
    if parent_tether_config == None:
        return True
    path1 = LineString([parent_tether_config[0],tether_config[0]])
    path2 = LineString([parent_tether_config[-1],tether_config[-1]])
    if distance(tether_config[0], tether_config[-1]) < 2:
        # print(1)
        return False
    if is_simple(LineString(tether_config)) == True:
        return True
    else: 
        # collect1 = []
        # collect2 = []
        for i in range(0,len(tether_config)-1):      
            for j in range(0,len(tether_config)-1):
                if i != j:
                    # print([tether_config[i],tether_config[i+1]])
                    line1 = LineString([tether_config[i],tether_config[i+1]])
                    line2 = LineString([tether_config[j],tether_config[j+1]])
                    line_intersect = intersection(line1, line2)
                    line_intersect = get_coordinates(line_intersect).tolist()
                    if line_intersect != []:
                        if compare(line_intersect, tether_config) == False:
                            # print(2)
                            return False
        tag1 = 0
        tag2 = 0 
        tether_config_ = tether_config_extend(tether_config)        
        for k in range(1,len(tether_config_)-1):
            line0 = LineString([tether_config_[k],tether_config_[k+1]])
            # print(path1)
            # print(line0)
            line_intersect = intersection(line0, path1)
            line_intersect = get_coordinates(line_intersect).tolist() 
            # print(line_intersect)
            if line_intersect != []:         
                return False      
                
        for k in range(0,len(tether_config_)-2):
            line0 = LineString([tether_config_[k],tether_config_[k+1]])
            # print(path1)
            # print(line0)
            line_intersect = intersection(line0, path2)
            line_intersect = get_coordinates(line_intersect).tolist() 
            # print(line_intersect)
            if line_intersect != []:           
                return False             
        
                
        for i in range(1,len(tether_config)-1):
            s = []
            for itr in range(1,len(tether_config)-1):
                if(tether_config[itr] == tether_config[i]):
                    s.append(itr)
            if len(s) >= 2:
                # print(i)
                # print(s)
                vector = []
                for idx in s:
                    vector.append([[tether_config[idx],tether_config[idx - 1]],[tether_config[idx],tether_config[idx + 1]]])
                # return 0
                # print(vector)
                for i in range(0, len(vector)):
                    vect1 = np.array(vector[i][0][1]) - np.array(vector[i][0][0])
                    vect2 = np.array(vector[i][1][1]) - np.array(vector[i][1][0])    
                    # print(vect1,vect2)                                   
                    for j in range(0, len(vector)):
                        if i != j:
                            vect3 = np.array(vector[j][0][1]) - np.array(vector[j][0][0])
                            vect4 = np.array(vector[j][1][1]) - np.array(vector[j][1][0])  
                            # print(vect3,vect4)         
                            angle1 = np.arctan2(vect1[1],vect1[0])
                            angle2 = np.arctan2(vect2[1],vect2[0])
                            angle3 = np.arctan2(vect3[1],vect3[0])
                            angle4 = np.arctan2(vect4[1],vect4[0])
                            # print(angle1,angle2,angle3,angle4)
                            if angle3 < max(angle1,angle2) and angle3 > min(angle1,angle2):
                                if angle4 > max(angle1,angle2) or angle4 < min(angle1,angle2):
                                    # print(1)
                                    # print(4)
                                    return False
                            if angle4 < max(angle1,angle2) and angle4 > min(angle1,angle2):
                                if angle3 > max(angle1,angle2) or angle3 < min(angle1,angle2):
                                    # print(2)
                                    # print(5)
                                    return False   
        for i in range(0,len(tether_config) - 1):
            for j in range(0,len(tether_config) - 1):
                if i != j:
                    if distance(tether_config[i],tether_config[j]) < 0.01:
                        if distance(tether_config[i+1], tether_config[j+1]) < 0.01:
                            # print(6)
                            return False
        # for i in range(0,len(tether_config)-1):
        #     for j in range(0,len(tether_config) - 2):
        #         if i != j:
        #             if distance(tether_config[i], tether_config[j+1]) < 0.01:
        #                 if distance(tether_config[i+1], tether_config[j]) < 0.01:
                            
        return True       
                                
def tether_not_cross(tether_config):
    if distance(tether_config[0], tether_config[-1]) < 2:
        # print(1)
        return False
    if is_simple(LineString(tether_config)) == True:
        return True
    else:
        return False                            
                            
# print(tether_not_cross([[0,0],[1,0],[2,0],[2,1],[1,1],[1,0],[2,0],[3,0]]))
                    
# parent_tether_config = [[0.0, 26.0], [5, 23], [6.7, 21], [6.1, 10], [6, 5], [12, 5], [15,21],[19,22],[20,17],[15,12],[16,9],[18,6],[19,12],[20,17],[20,24],[18,24]]             
# tether_config =        [[0.0, 26.0], [5, 23], [6.7, 21], [6.1, 10], [6, 5], [12, 5], [15,21],[19,22],[20,17],[15,12],[16,9],[18,6],[19,12],[20,17],[19,22],[18,24]]

# parent_tether_config = [[0.0, 26.0], [5, 23], [6.7, 21], [6.1, 10], [6, 5], [12, 5], [15.2, 12], [17, 13], [19.6, 12], [18, 6], [12, 5], [6, 5], [1.051316701949486, 16.683772233983163]]       
# tether_config = [[0.0, 26.0], [5, 23], [6.7, 21], [6.1, 10], [6, 5], [12, 5], [15.2, 12], [17, 13], [19.6, 12], [18, 6], [12, 5], [6, 5], [6.1, 10],[15.0, 16.0]]   
 

# parent_tether_config = [[7.46193931775946, 21.64764842009554], [6.1, 10], [6, 5], [12, 5], [15.2, 12], [17, 13], [19.6, 12], [18, 6], [12, 5], [6, 5], [5.100199940019992, 10.0199960011996]]
# tether_config =       [[12.900883074863264, 10.434061845161391], [6.1, 10], [6, 5], [12, 5], [15.2, 12], [17, 13], [19.6, 12], [18, 6], [12, 5], [6, 5], [6.1, 10], [6.962306564570794, 16.72803295092244]]         
# parent_tether_config = [[0.0, 26.0], [5, 23], [6.7, 21], [6.1, 10], [6, 5], [12, 5], [15.2, 12], [17, 13], [19.6, 12], [18, 6], [12, 5], [6, 5], [1.051316701949486, 16.683772233983163]]
# tether_config =        [[0.0, 26.0], [5, 23], [6.7, 21], [6.1, 10], [6, 5], [12, 5], [15.2, 12], [17, 13], [19.6, 12], [18, 6], [12, 5], [6, 5], [6.1, 10], [17.13103409656997, 13.991377862137385]]

# parent_tether_config = [[16.734637935507187, 13.964148834323888], [6.1, 10], [6, 5], [12, 5], [18, 6], [19.6, 12], [17.35897907930887, 13.93334560620306]]
# tether_config = [[14.714357068821368, 12.874157276121538], [6.1, 10], [6, 5], [12, 5], [18, 6], [19.6, 12], [17, 13], [14.714357068821368, 12.874157276121538]]
# print(tether_not_cross_winding(tether_config,parent_tether_config))
                
        

                
            
            

    
def tether_winding_angle(tether_config):
    tether_config = np.array(tether_config)
    if len(tether_config) == 2:
        return 0
    if len(tether_config) > 2:
        angle_sum = 0
        for i in range(1,len(tether_config) - 1):
            v1 = tether_config[i] - tether_config[i-1]  
            v2 = tether_config[i+1] - tether_config[i]
            dot_product = np.dot(v1,v2)/(np.linalg.norm(v1)*np.linalg.norm(v2))
            if dot_product > 1:
                dot_product = 1
            if dot_product < -1:
                dot_product = -1
            winding_angle = np.arccos(dot_product)
            angle_sum = winding_angle + angle_sum
        return angle_sum
    

    

        


# print(tether_not_cross(tether_config))
# line = LineString([(0, 0), (2, 2)])
# a = intersection(line, LineString([(0,0), (2,2)]))   
# print(get_coordinates(a).tolist())

# b = [[0,0],[0,1],[1,0],[1,1],[0.5,1],[0,0]]

# my_list = b
 
# # find length of the list
# list_size = len(my_list)
# # declare for loop
# s = []
# for itr in range(list_size):
#     if(my_list[itr] == [0,0]):
#         s.append(itr)
# print(s)



# Tether_config, length = tether_update(start_pos, end_pos, tether_config, obstacle)
# print(Tether_config, length)

# fig,ax = plt.subplots()
# for i in obstacle:
#     p = P(i, facecolor = 'k')
#     ax.add_patch(p)
# plt.plot([i[0] for i in tether_config],[i[1] for i in tether_config],'r')
# plt.plot([i[0] for i in Tether_config],[i[1] for i in Tether_config],'b')
# ax.set_xlim([0,30])
# ax.set_ylim([0,30])
# plt.show()

        

             
            
            
            
        
            
    


    


    
 
 
 
 
 
 
 
 
 
 
        

