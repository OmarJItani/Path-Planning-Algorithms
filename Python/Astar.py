from random import uniform
import numpy as np
import matplotlib.pyplot as plt
import math
from queue import PriorityQueue

class Graph():
    def __init__(self, map_dim, obstacle_intensity):
        # store map dimension and obstacle intensity
        self.map_dim = map_dim
        self.obstacle_intensity = obstacle_intensity

        # create the map with random obstacles
        self.map = np.array([[np.array([1.,1.,1.]) if uniform(0, 1) >= self.obstacle_intensity else np.array([0.,0.,0.]) for i in range(self.map_dim)] for j in range(self.map_dim)])

        # randomly select initial and goal positions
        self.goal_position = [int(uniform(0, self.map_dim-1)) , int(uniform(0, self.map_dim-1))]
        self.initial_position = [int(uniform(0, self.map_dim-1)) , int(uniform(0, self.map_dim-1))]

        # Ensure that initial and goal positions are not spawned on obstacles
        self.map[self.initial_position[0],self.initial_position[1]] = np.array([1.,1.,1.])
        self.map[self.goal_position[0],self.goal_position[1]] = np.array([1.,1.,1.])

    def draw_map(self, pathsol, visited_nodes):
        """
        Draws the map with the resultant path.
        
        Input: 
        pathsol: Path resulting from the Astar function.
        visited_nodes: List of visited nodes
        """

        # draw visited nodes in gray
        for i in visited_nodes[:-2]:
            self.map[i.position[0],i.position[1],0] = 0.5
            self.map[i.position[0],i.position[1],1] = 0.5
            self.map[i.position[0],i.position[1],2] = 0.5


        # draw path in blue on map
        for i in pathsol[1:-1]:
            self.map[i.position[0],i.position[1],0] = 0
            self.map[i.position[0],i.position[1],1] = 0
            self.map[i.position[0],i.position[1],2] = 1

        # make goal position red on map
        self.map[self.goal_position[0],self.goal_position[1],0] = 1
        self.map[self.goal_position[0],self.goal_position[1],1] = 0
        self.map[self.goal_position[0],self.goal_position[1],2] = 0

        # make initial position green on map
        self.map[self.initial_position[0],self.initial_position[1],0] = 0
        self.map[self.initial_position[0],self.initial_position[1],1] = 1
        self.map[self.initial_position[0],self.initial_position[1],2] = 0

        # plot the map
        map2 = np.flipud(np.transpose(self.map,(1,0,2)))
        plt.imshow(map2,cmap='gray',vmin=0, vmax=1)  # grayscale: gray / rgb: brg

        ax = plt.gca()
        ax.set_xticks(np.arange(-.5, self.map_dim, 1), minor=True)
        ax.set_yticks(np.arange(-.5, self.map_dim, 1), minor=True)
        ax.grid(which='minor', color='w', linestyle='-', linewidth=0.1)

        plt.show()

    def draw_map_dynamic(self, pathsol, visited_nodes):
        """
        Draws the map dynamically.
        
        Input: 
        pathsol: Path resulting from the Astar function.
        visited_nodes: List of visited nodes
        """

        # make goal position red on map
        self.map[self.goal_position[0],self.goal_position[1],0] = 1
        self.map[self.goal_position[0],self.goal_position[1],1] = 0
        self.map[self.goal_position[0],self.goal_position[1],2] = 0

        # make initial position green on map
        self.map[self.initial_position[0],self.initial_position[1],0] = 0
        self.map[self.initial_position[0],self.initial_position[1],1] = 1
        self.map[self.initial_position[0],self.initial_position[1],2] = 0

        plt.ion()
        plt.show()

        ax = plt.gca()
        ax.set_xticks(np.arange(-.5, self.map_dim, 1), minor=True)
        ax.set_yticks(np.arange(-.5, self.map_dim, 1), minor=True)
        # ax.grid(which='minor', color='k', linestyle='-', linewidth=0.5)

        # draw visited nodes in gray
        for i in visited_nodes[1:-1]:
            self.map[i.position[0],i.position[1],0] = 0.5
            self.map[i.position[0],i.position[1],1] = 0.5
            self.map[i.position[0],i.position[1],2] = 0.5

            plt.clf()
            map2 = np.flipud(np.transpose(self.map,(1,0,2)))
            plt.imshow(map2,cmap='gray',vmin=0, vmax=1)  # grayscale: gray / rgb: brg
            plt.pause(0.0001)

        # draw path in blue on map
        for i in pathsol[1:-1]:
            self.map[i.position[0],i.position[1],0] = 0
            self.map[i.position[0],i.position[1],1] = 0
            self.map[i.position[0],i.position[1],2] = 1
        
        plt.ioff()
        
        map2 = np.flipud(np.transpose(self.map,(1,0,2)))
        plt.imshow(map2,cmap='gray',vmin=0, vmax=1)  # grayscale: gray / rgb: brg
        
        plt.show()


class node():
    nodes_lst = [] # list of all created nodes
    def __init__(self, position, graph, g_cost = 0, h_cost = 0):
        self.position = position
        self.neighbours = []
        self.parent = None
        self.g_cost = g_cost
        self.h_cost = h_cost
        self.graph = graph

        # add the created instance to the list of all nodes
        node.nodes_lst.append(self)

    # Allows to compare the nodes based on the values of their costs
    def __lt__(self, other):
        return self.g_cost + self.h_cost <= other.g_cost + other.h_cost

    # Represent the node as a string: "Node [ x , y]"
    def __repr__(self):
        return f'Node {self.position}'

    def get_neighbours(self):
        """
        Gets the neighbour of a node object.
        """
        for i in range(0,3):
            for j in range(0,3):
                # check if the neighbouring node to be created is inside the map boundary
                if self.position[0]+i-1 >= 0 and self.position[0]+i-1 <= self.graph.map_dim-1 and self.position[1]+j-1 >= 0 and self.position[1]+j-1 <= self.graph.map_dim-1:

                    # check if the neighbouring node already exists
                    node_already_exists = 0
                    for every_node in node.nodes_lst:
                        if every_node.position == [self.position[0]+i-1,self.position[1]+j-1]:
                            node_already_exists = 1
                            if (i!=1 and j!=1): # neighbouring node is not the parent node
                                self.neighbours.append( every_node )
                            break

                    # only consider creating a node if it does not exist
                    if node_already_exists == 0:
                        # checks if the considered neighbouring node position is same as its parent position or is an obstacle
                        if (i==1 and j==1) or (self.graph.map[self.position[0]+i-1, self.position[1]+j-1, :] == 0.).all():
                            # if yes, pass
                            pass
                        else:
                            # else, create neighbouring node
                            self.neighbours.append( node([self.position[0]+i-1,self.position[1]+j-1], self.graph, 1000, 1000) )




def A_star(graph):
    """
    Implements Astar algorithm (finds the shortest path connecting an initial position to a goal position on a map).
    
    Input:
    graph: A graph instance that has the considered map, initial position, and goal position as attributes.
    
    Output:
    pathsol: Optimal path connecting the initial position to the goal position.
    """

    def dist(lst1,lst2):
        """
        Computes the eulidean distance between two points
        
        Inputs:
        lst1: First point [x1 , y1]
        lst2: Second point [x2 , y2]
        
        Output:
        dist: Euclidean distance
        """

        dist = math.sqrt( (lst1[0]-lst2[0])**2 + (lst1[1]-lst2[1])**2 )
        return dist

    closed_list = []
    open_list = PriorityQueue()
    open_list.put( node(graph.initial_position, graph) )

    while(open_list.qsize()):

        current_node = open_list.queue[0] # get element with best cost but keep it in open_list

        # if current_node is the goal node, extract and return path
        if current_node.position == graph.goal_position:
            path = [current_node]

            while (current_node.parent is not None):
                path.insert(0,current_node.parent)
                current_node = current_node.parent
            return path , closed_list

        else:
            if current_node.neighbours == []:
                current_node.get_neighbours()

            for i in current_node.neighbours:
                neighbour_cost = current_node.g_cost + dist(current_node.position,i.position)

                if i in open_list.queue:
                    if i.g_cost <= neighbour_cost:
                        pass
                    else:
                        i.g_cost = neighbour_cost
                        i.parent = current_node

                elif i in closed_list:
                    if i.g_cost <= neighbour_cost:
                        pass
                    else:
                        ind = closed_list.index(i)
                        to_be_added_to_open = closed_list.pop(ind)
                        if to_be_added_to_open not in open_list.queue:
                            open_list.put(to_be_added_to_open)
                        i.g_cost = neighbour_cost
                        i.parent = current_node

                else:
                    if i not in open_list.queue:
                        open_list.put(i)
                    i.h_cost = dist(i.position, graph.goal_position)
                    i.g_cost = neighbour_cost
                    i.parent = current_node

            to_be_added_to_closed = open_list.get()
            if to_be_added_to_closed not in closed_list:
                closed_list.append(to_be_added_to_closed)
    
    # no solution
    return [] , closed_list


def main():
    # Define the dimension of the map
    map_dim = 40
    # Define the intensity percentage of the map obstacles
    obstacle_intensity = 0.05

    # Create graph
    graph = Graph(map_dim, obstacle_intensity)

    # Find optimal path
    pathsol , visited_nodes = A_star(graph)

    # Visualize solution
    graph.draw_map(pathsol, visited_nodes)
    # graph.draw_map_dynamic(pathsol, visited_nodes)

if __name__ == '__main__':
    main()