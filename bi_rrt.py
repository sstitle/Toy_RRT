from Geometry import *
import matplotlib.pyplot as plt
import numpy as np
import random
import time
import warnings


class TreeNode:
    def __init__(self,parent_index,position_vector):
        self.parent = parent_index
        self.pos = position_vector

def buildRrt(obstacles,q_init,q_goal, max_iters=100, max_step=0.5):
    #takes in the starting pose, number of iterations to build the RRT, and the max distance to increment by
    #returns the built tree
    tree = [TreeNode(0,q_init)]
    for i in range(max_iters):
        q_rand = randFreeConfiguration(obstacles) #random sample in free configuration space
        #plt.plot(q_rand.x,q_rand.y,'go')
        #print debug = "Random pose generated at: {}".format(q_rand)
        near_idx = nearest_vertex(q_rand,tree)
        q_near = tree[near_idx].pos
        #print "nearest node is at {}".format(q_near)
        q_add = newConfiguration(q_near,q_rand,max_step)
        if not EdgeCollision(q_add,q_near,obstacles):
            node = TreeNode(near_idx,q_add)
            tree.append(node)
            PlotSeg(q_add,q_near)
            #plt.plot(node.pos.x,node.pos.y,'go')
            #plt.pause(0.1)
            #while we're adding an edge, let's see if we can reach the goal
            if not EdgeCollision(q_goal,q_add,obstacles):
                parent = len(tree)-1
                goal_node = TreeNode(parent,q_goal)
                tree.append(goal_node)
                PlotSeg(q_add,q_goal)
                # print "Goal Reached!"
                break;
    return tree

def buildBiRrt(obstacles,q_init,q_goal,max_iters=100, max_step=0.5):
    #takes in the starting and goal pose, obstacles, number of iterations and max step size
    start_tree = [TreeNode(0,q_init)]
    goal_tree = [TreeNode(0,q_goal)]
    succ = False
    for i in range(max_iters):
        #print "Iteration {}".format(i)
        q_rand = randFreeConfiguration(obstacles)

        #both trees will find the nearest vertex to the sampled point
        near_idx_start = nearest_vertex(q_rand,start_tree)
        near_idx_goal = nearest_vertex(q_rand,goal_tree)
        #find the positions of those nearest vertices
        q_near_s = start_tree[near_idx_start].pos
        q_near_g = goal_tree[near_idx_goal].pos

        #creates a new configuration within a certain distance of the nearest tree vertex
        q_add_s = newConfiguration(q_near_s,q_rand,max_step)
        q_add_g = newConfiguration(q_near_g,q_rand,max_step)

        #stop = 0 #variable which increments if a tree connects, if both trees connect, we can stop
        #if the edge doesn't cause a collision, add it to the current tree...
        if not EdgeCollision(q_add_s,q_near_s,obstacles):
            node = TreeNode(near_idx_start,q_add_s)
            start_tree.append(node)
            PlotSeg(q_add_s,q_near_s)

        if not EdgeCollision(q_add_g,q_near_g,obstacles):
            node = TreeNode(near_idx_goal,q_add_g)
            goal_tree.append(node)
            PlotSeg(q_add_g,q_near_g)

        if q_near_s is q_near_g:
            print "Success! A path from start to goal was found."
            succ = True
            break
        iters = i
    if not succ:
        print "Failure! A path from start to goal was not found. Try increasing the number of iterations. Otherwise, a viable path may not exist."
    return (succ,start_tree,goal_tree,iters)





def newConfiguration(q_near,q_rand,max_step):
    #returns a point within the step distance of the input random pose
    if Distance(q_rand,q_near) < max_step:
        q_new = q_rand
    else:
        u = max_step/Distance(q_rand,q_near)
        q_new = InterpolateLinear(u,q_near,q_rand)
    return q_new



def nearest_vertex(q_target,tree):
    #returns the index of the nearest pose/vertex in the tree to the randomly chosen pose
    cnt = 0
    min_dst = 3;
    min_idx = None;
    for node in tree:
        dst = Distance(node.pos,q_target)
        if dst < min_dst:
            min_dst = dst
            min_idx = cnt
        cnt = cnt + 1
    return min_idx


def randFreeConfiguration(obstacles):
    #returns a random sample from the free part of the configuration space
    q_rand = None
    sample = Vector(np.random.random(),np.random.random())
    while NodeCollision(sample,obstacles):
        sample = Vector(np.random.random(),np.random.random())
    q_rand = sample
    return q_rand

def extractPath(rrt_input):
    #inputs a tree and extracts the path from start to goal
    #returns a list of the vertices of the path from start to goal found by the rrt
    node = rrt_input[-1]
    path = [node.parent]
    path_vrts = [node.pos]
    run_bool = True
    while run_bool is True:
        node = rrt_input[node.parent]
        path.append(node.parent)
        path_vrts.append(node.pos)
        if node.parent is 0:
            run_bool = False
            path_vrts.append(rrt_input[node.parent].pos)
    #PlotSeg(rrt_input[-1].pos,rrt_input[-2].pos,'r-')
    # for i in range(len(path)-1):
    #     PlotSeg(rrt_input[path[i]].pos,rrt_input[path[i+1]].pos,'r-')
    ###### Debug Lines
    print "Extracted Path:"
    j = 0
    for vertex in path_vrts:
        print "{} {}".format(j,vertex)
        j += 1
    print "\n"
    ######
    return path_vrts

def pickTwoPoints(search_path):
    path_length = 0
    edge_lengths = []
    #print "Picking two points:"
    for j in range(len(search_path)-1):
        #print search_path[j]
        path_length += Distance(search_path[j],search_path[j+1])
        edge_lengths.append(path_length)
    #print edge_lengths
    r1 = np.random.random()*path_length
    r2 = np.random.random()*path_length
    if r1 > r2:
        temp = r1
        r1 = r2
        r2 = temp
    cnt1 = 0
    while edge_lengths[cnt1] < r1:
        cnt1 += 1
    if cnt1 is 0:
        u = r1
    else:
        u = edge_lengths[cnt1] - r1 #value for interpolating
    pnt1 = InterpolateLinear(u,search_path[cnt1],search_path[cnt1+1])
    cnt2 = 0
    while edge_lengths[cnt2] < r2:
        cnt2 += 1
    if cnt2 is 0:
        u = r2
    else:
        u = edge_lengths[cnt2] - r2
    pnt2 = InterpolateLinear(u,search_path[cnt2],search_path[cnt2+1])
    points = (pnt1,pnt2,cnt1,cnt2)
    # ###### Debug Lines
    # print "interpolating b/w {} and {}".format(search_path[cnt1],search_path[cnt1+1])
    # print "interpolated point is {}".format(points[0])
    # print "interpolating b/w {} and {}".format(search_path[cnt2],search_path[cnt2+1])
    # print "interpolated point is {}".format(points[1])
    # plt.plot(points[0].x,points[0].y,'g*')
    # plt.plot(points[1].x,points[1].y,'k*')
    # ######
    return points

def shortcut(input_path,obst,iters=10000):
    coll = True
    for i in range(iters):
    # i = 0
    # while coll:
    #     i += 1
    #     print i
        new_vert1, new_vert2, cnt1, cnt2 = pickTwoPoints(input_path)
        #pick two random points uniformly along path length
        coll = EdgeCollision(new_vert1,new_vert2,obst)
            # print coll
        if not coll and cnt1 is not cnt2:
            # plt.plot(new_vert1.x,new_vert1.y,'bo')
            # plt.plot(new_vert2.x,new_vert2.y,'ko')
            # PlotSeg(new_vert1,new_vert2,'b-')
            #print "Iteration {}".format(i)
            #print "Remove vertices between {} and {}".format(cnt1, cnt2)
            new_path = []
            new_path = input_path[:cnt1+1]
            new_path += [new_vert1]
            new_path += [new_vert2]
            new_path += input_path[cnt2+1:]
            input_path = new_path

    for l in range(len(input_path)-1):
        # plt.plot(input_path[l].x,input_path[l].y,'r*')
        PlotSeg(input_path[l],input_path[l+1],'g-')
    j = 0
    for vertex in input_path:
        print "{} {}".format(j,vertex)
        j += 1
    return input_path

def simplifyPath(complex_path,obst):
    for i in range(1,len(complex_path)):    #for each point
        print "Iteration {}".format(i)
        for point in complex_path:
            print point
        j = 1
        while (i+j < len(complex_path)):    #look for other points
            if not EdgeCollision(complex_path[i],complex_path[i+j],obst):
                j += 1
            else:
                print "J vertices to be skipped {}".format(j)
                if (j > 1):
                    new_path = []
                    new_path = complex_path[:i]
                    new_path += complex_path[i+j-1:]
                    complex_path = new_path
            print "i is {} and ij is {} and length of path is {}".format(i,i+j,len(complex_path))
            if i+j == len(complex_path)-1: #got to the goal without hitting an object
                new_path = []
                new_path = complex_path[:i]
                new_path = complex_path[i+j-1]
                complex_path = new_path

    print complex_path
    for l in range(len(complex_path)-1):
        plt.plot(complex_path[l].x,complex_path[l].y,'g*')
        PlotSeg(complex_path[l],complex_path[l+1],'b-')

    print "Vertices in simplified path: {}".format(len(complex_path))
    return


if __name__ == "__main__":
    # obstacles 1
    # obstacles = []
    # obstacles.append((Vector(0.175, 0.6), Vector(0.05, 0.3)))
    # obstacles.append((Vector(0.5, 0.375), Vector(0.05, 0.75)))
    # obstacles.append((Vector(0.825, 0.6), Vector(0.05, 0.30)))
    # obstacles.append((Vector(0.5, 0.725), Vector(0.70, 0.05)))
    # #start and goal pose
    # q_start = Vector(0.3333, 0.6)
    # q_goal  = Vector(0.6666, 0.6)

    #obstacles 2
    obstacles = []
    obstacles.append((Vector(0.5,0.5),Vector(0.3,0.3)))
    q_start = Vector(0.01,0.01)
    q_goal = Vector(0.99,0.99)

    # #obstacles 3
    # obstacles = []
    # obstacles.append((Vector(0.2,0.4),Vector(0.05,0.8)))
    # obstacles.append((Vector(0.4,0.6),Vector(0.05,0.8)))
    # obstacles.append((Vector(0.6,0.4),Vector(0.05,0.8)))
    # obstacles.append((Vector(0.8,0.6),Vector(0.05,0.8)))
    # q_start = Vector(0.01,0.01)
    # q_goal = Vector(0.99,0.99)

    # #obstacles 4
    # obstacles = []
    # obstacles.append((Vector(0.2,0.4),Vector(0.05,0.8)))
    # obstacles.append((Vector(0.5,0.8),Vector(0.65,0.05)))
    # obstacles.append((Vector(0.8,0.5),Vector(0.05,0.6)))
    # obstacles.append((Vector(0.55,0.2),Vector(0.5,0.05)))
    # obstacles.append((Vector(0.3,0.45),Vector(0.05,0.45)))
    # q_start = Vector(0.01,0.01)
    # q_goal = Vector(0.5,0.5)

    fig, ax = plt.subplots(1)

    PlotRects(obstacles, ax)

    plt.gca().set_xlim(0,1)
    plt.gca().set_ylim(0,1)
    plt.gca().set_aspect('equal')
    passed, tree1, tree2, iters = buildBiRrt(obstacles,q_start,q_goal,4000,0.4)
    if passed:
        found_path1 = extractPath(tree1)
        found_path2 = extractPath(tree2)
        final_path = found_path1[::-1] + found_path2[1:]


        for i in range(len(final_path)-1):
             PlotSeg(final_path[i],final_path[i+1],'r-')

        smooth_path = shortcut(final_path,obstacles)
        simplifyPath(smooth_path,obstacles)
        # for v in smooth_path:
        #   plt.plot(v.x,v.y,'r*')

    plt.plot(q_start.x,q_start.y,'b*')
    plt.plot(q_goal.x ,q_goal.y ,'r*')
    plt.show()
