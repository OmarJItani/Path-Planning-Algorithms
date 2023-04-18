from random import uniform
import numpy as np
import matplotlib.pyplot as plt
import math

class Graph():
    def __init__(self, map_dim, obstacle_intensity):
        # store map dimension and obstacle intensity
        self.map_dim = map_dim
        self.obstacle_intensity = obstacle_intensity
        
        # create create the map with random obstacles
        self.map = np.array([[np.array([1.,1.,1.]) if uniform(0, 1) >= self.obstacle_intensity else np.array([0.,0.,0.]) for i in range(self.map_dim)] for j in range(self.map_dim)])
        
        # randomly select initial and goal positions
        self.goal_position = [int(uniform(0, self.map_dim-1)) , int(uniform(0, self.map_dim-1))]
        self.initial_position = [int(uniform(0, self.map_dim-1)) , int(uniform(0, self.map_dim-1))]

    def draw_map(self, pathsol, visited_nodes):
        """
        Draws the map with the resultant path.
        
        Input: 
        pathsol: Path resulting from the Dijkstra function.
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
    def __init__(self, position, graph, cost = 0):
        self.position = position
        self.neighbours = []
        self.parent = None
        self.cost = cost
        self.graph = graph

        # add the created instance to the list of all nodes
        node.nodes_lst.append(self)

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
                            self.neighbours.append( node([self.position[0]+i-1,self.position[1]+j-1], self.graph, 1000) )
        



def Dijkstra(graph):
    """
    Implements the Dijkstra algorithm (finds the shortest path connecting an initial position to a goal position on a map).
    
    Input:
    graph: A graph instance that has the considered map, initial position, and goal position as attributes.
    
    Output:
    pathsol: Optimal path connecting the initial position to the goal position.
    closed_lst: A list of all visited nodes
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
    open_list = [node(graph.initial_position, graph)]

    while(len(open_list) > 0):

        least_cost = 10000
        for i in open_list:
            if i.cost < least_cost:
                least_cost = i.cost
                least_node = open_list.index(i)
        current_node = open_list[least_node]
        
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
                neighbour_cost = current_node.cost + dist(current_node.position, i.position)
                
                if i in open_list:
                    if i.cost <= neighbour_cost:
                        pass
                    else:
                        i.cost = neighbour_cost
                        i.parent = current_node
                
                elif i in closed_list:
                    if i.cost <= neighbour_cost:
                        pass
                    else:
                        ind = closed_list.index(i)
                        to_be_added_to_open = closed_list.pop(ind)
                        if to_be_added_to_open not in open_list:
                            open_list.append(to_be_added_to_open)
                        i.cost = neighbour_cost
                        i.parent = current_node

                else:
                    if i not in open_list:
                        open_list.append(i)
                    i.cost = neighbour_cost
                    i.parent = current_node
            
            to_be_added_to_closed = open_list.pop(open_list.index(current_node))
            if to_be_added_to_closed not in closed_list:
                closed_list.append(to_be_added_to_closed)


def main():
    # Define the dimension of the map
    map_dim = 40
    # Define the intensity percentage of the map obstacles
    obstacle_intensity = 0.05

    # Create graph
    graph = Graph(map_dim, obstacle_intensity)

    # Find optimal path
    pathsol , visited_nodes = Dijkstra(graph)
    print(f'The path is: {pathsol}')
    
    # Visualize solution
    graph.draw_map(pathsol, visited_nodes)

if __name__ == '__main__':
    main()
