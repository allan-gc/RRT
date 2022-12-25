import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection
import numpy as np
import random
import math

class GenRRT:

    def __init__(self,q_in,k,disp,d):
        self.q_in=q_in
        self.k=k
        self.disp=disp
        self.d=d
    
    def make_rrt(self,q_in, k, disp, d,circle_radii, circle_centers,end):
        """ Calculates k rtt nodes starting from q_init node position. """
        k=np.array(range(k))
        g=[q_in]
        temp_dist=[]
        parent_list=[]
        childlist=[]
        count=0
        for randvertex in k:
            q_rand=random_config(d)
            for node in g:
                count+=1
                treedist=(nearest_vertex(q_rand,node))
                temp_dist.append(treedist)
                if count==len(g):
                    min_dist_index=temp_dist.index(min(temp_dist)) 
                    q_near=g[min_dist_index]
                    temp_dist=[]
                    q_new=new_configuration(q_near,q_rand,disp)
                    intersect_check=find_goal(q_near,q_new,circle_radii,circle_centers)
                    if len(intersect_check)!=0:
                        q_new=0
                        count=0   
                    else:
                        q_new=q_new

            if q_new!=0:
                g.append(q_new)
                parent_list.append(q_near)
                childlist.append(q_new)
                count=0    

            if q_new!=0:
                check_end=find_goal(q_new,end,circle_radii,circle_centers)
                if len(check_end)==0:
                    q_near=q_new
                    q_new=end
                    g.append(q_new)
                    parent_list.append(q_near)
                    childlist.append(q_new)
                    break

        return g, parent_list, childlist


    def make_plot(self,g,parent,child,start,end):
            """ Plots rrt nodes. """
            plt.axis([0,100,0,100])
            x_init,y_init=zip(start)
            x_goal,y_goal=zip(end)
            x,y=zip(*g)

            plt.scatter(x_init,y_init,90,color='green')
            plt.scatter(x_goal,y_goal,90,color='red',marker="X")
            for index in range(len(parent)):
                ax.scatter(x[index], y[index], 5,color='blue')
                lctree=LineCollection([[parent[index],child[index]]])
                ax.add_collection(lctree)   
                plt.pause(0.005)



def find_goal(qnew,end,circle_radii,circle_centers):
    """ 
        Checks if the newly added point on the tree is within a circle obtacle. 
        Also checks if there is a clear path between the newly added point on
        the tree and the goal.
    
    """
    d=[end[0]-qnew[0],end[1]-qnew[1]]
    check_list=[]
    for circle in range(len(circle_radii)):
        f=[qnew[0]-circle_centers[circle][0],qnew[1]-circle_centers[circle][1]]
        a=np.dot(d,d)
        b=2*np.dot(f,d)
        c=np.dot(f,f) - (circle_radii[circle]**2)
        check=0
        disc = (b**2) - 4*a*c
        if disc<=0:
            check=0
        else:
            sqrtdisc = math.sqrt(abs(disc))
            t1 = (-b + sqrtdisc)/(2*a)
            t2 = (-b - sqrtdisc)/(2*a)
    
            if( t1 >= 0 and t1 <= 1 ):
                check=1
                check_list.append(check)
    
            if( t2 >= 0 and t2 <= 1 ):
                check=1
                check_list.append(check)

    return check_list


def find_path(parents,children,ax):
    """ Creates path from random child node to q_init. """
    
    random_child=(len(children)-1)
    current_child=children[random_child]
    current_parent=parents[random_child]
    init_node=[]
    child_path=[current_child]
    parent_path=[current_parent]


    if current_parent!=parents[0]:
        child_index_of_parent=children.index((parents[random_child])) 
        while len(init_node)==0:
            current_child=children[child_index_of_parent]
            current_parent=parents[child_index_of_parent]
            child_path.append(current_child)
            parent_path.append(current_parent)
            if current_parent == parents[0]:
                init_node.append(parents[0])
            else:
                child_index_of_parent=children.index((parents[child_index_of_parent]))
     
    for index in range(len(child_path)):
        lcpath=LineCollection([[parent_path[index],child_path[index]]],color='red')
        ax.add_collection(lcpath) 
        plt.pause(0.0005)


def make_circle(n,max_r,d,ax):
    """ Makes random number of obstacles of varius sizes. """
    plt.axis([0,100,0,100])
    circle_radii=[]
    circle_centers=[]
    for index in range(n):
        circle=plt.Circle((np.random.randint(0,d[0][1]), np.random.randint(0,d[1][1])),np.random.randint(0,max_r) ,color='k')
        ax.add_patch(circle)  
        circle_radii.append(circle.radius)
        circle_centers.append(circle.center)
        ax.add_patch(circle)

    return circle_radii,circle_centers


def random_init(circle_radii, circle_centers,d):
    """ Generates random start and end points, checks if they are within circle obstacles. """ 
    random_qinit=[np.random.randint(0,d[0][1]), np.random.randint(0,d[1][1])]
    random_goal=[np.random.randint(0,d[0][1]), np.random.randint(0,d[1][1])]
    x_init,y_init=zip(random_qinit)
    x_goal,y_goal=zip(random_goal)

    for circle in range(len(circle_radii)):
        qinit_distance=math.dist(random_qinit,circle_centers[circle])
        goal_distance=math.dist(random_goal,circle_centers[circle])
        if qinit_distance <= circle_radii[circle]:
            while qinit_distance<=circle_radii[circle]:
                random_qinit=[np.random.randint(0,d[0][1]), np.random.randint(0,d[1][1])]
                qinit_distance=math.dist(random_qinit,circle_centers[circle])
                x_init,y_init=zip(random_qinit)

        if goal_distance <= circle_radii[circle]:  
            while goal_distance<= circle_radii[circle]:
                random_goal=[random.uniform(0,d[0][1]), random.uniform(0,d[1][1])] 
                goal_distance=math.dist(random_goal,circle_centers[circle])
                x_goal,y_goal=zip(random_goal) 


    return random_qinit, random_goal


def random_config(d):
    """ Configures random position in domain. """
    rand_point=[random.uniform(0,d[0][1]), random.uniform(0,d[1][1])]
    return rand_point

def nearest_vertex(qrand,g):
    """ Finds the nearest vertex to the random position in configuration. """
    calc_dist=math.dist(qrand,g)
    return calc_dist
    
def new_configuration(qnear,qrand,disp):

    """ Calculates the unit vector from the nearest vertex to the random position, creates qnew. """
    vec_dist=[qrand[0]-qnear[0],qrand[1]-qnear[1]]
    norm=math.sqrt(vec_dist[0] ** 2 + vec_dist[1] **2)
    direction=[vec_dist[0]/norm, vec_dist[1]/norm]
    qnew=[qnear[0]+(disp*direction[0]), qnear[1]+(disp*direction[1])]

    return qnew


fig, ax= plt.subplots()
ax.set_title('RRT Path Finder')
fig.set_size_inches(10,8)
plt.ion()
circle_radii_array, circle_centers_array=make_circle(70,7,[[0,100],[0,100]],ax)
start,end=random_init(circle_radii_array,circle_centers_array,[[0,100],[0,100]])

#Create RRT object
rttobj=GenRRT(start,3000,1,[[0,100],[0,100]])
tree, parents, children=rttobj.make_rrt(start,3000,1,[[0,100],[0,100]],circle_radii_array,circle_centers_array,end)

rttobj.make_plot(tree,parents,children,start,end)
find_path(parents,children,ax)
plt.pause(10)


