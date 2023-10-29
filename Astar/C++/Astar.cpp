#include <iostream>
#include <vector>
#include <climits>
#include <algorithm>
#include <cmath>

using namespace std;

struct Graph {
    int map_dim;
    float obstacle_intensity;
    vector<vector<vector<float>>> map;
    vector<int> goal_position;
    vector<int> initial_position;

    Graph(int map_dim_v, float obstacle_intensity_v){
        map_dim = map_dim_v;
        obstacle_intensity = obstacle_intensity_v;
        select_initial_and_goal_positions();
        generate_map();
    }

    void select_initial_and_goal_positions(){

        srand(time(0));    // Using srand() with time(0) to change the random values everytime

        // Randomly select initial and goal positions
        initial_position = { rand() % (map_dim) , rand() % (map_dim) };
        goal_position = { rand() % (map_dim) , rand() % (map_dim) };
    };

    void generate_map(){ 
        
        // Create the map with random obstacles
        srand(time(0));    // Using srand() with time(0) to change the random values everytime
        map.resize(map_dim, vector<vector<float>>(map_dim, vector<float>(3)));
        for (int i = 0; i < map_dim; ++i) {
            for (int j = 0; j < map_dim; ++j) {
                if ((float)rand() / RAND_MAX >= obstacle_intensity) { map[i][j] = {1.0, 1.0, 1.0}; } 
                else { map[i][j] = {0.0, 0.0, 0.0}; }
            }
        }

        // Ensure that initial and goal positions are not spawned on obstacles
        map[initial_position[0]][initial_position[1]] = {1.0 ,1.0 ,1.0};
        map[goal_position[0]][goal_position[1]] = {1.0 ,1.0 ,1.0};
    };

    void color_and_print_map(vector<pair<int,int>> pathsolpair, vector<pair<int,int>> visitednodespair) {

        pair<int,int> curr_pair;
        //color the visited nodes
        for(int i=0 ; i<visitednodespair.size(); i++){
            curr_pair = visitednodespair[i];
            map[curr_pair.first][curr_pair.second] = {0.5, 0.5, 0.5};
        }

        // color the initial and goal positions
        map[initial_position[0]][initial_position[1]] = {0.0, 1.0, 0.0};
        map[goal_position[0]][goal_position[1]] = {1.0, 0.0, 0.0};

        //color the path
        for(int i=1 ; i<pathsolpair.size()-1; i++){
            curr_pair = pathsolpair[i];
            map[curr_pair.first][curr_pair.second] = {0.0 ,0.0 ,1.0};
        }

        // Print the generated map
        for (int i = 0; i < map_dim; ++i) {
            for (int j = 0; j < map_dim; ++j) {
                cout << "[" << map[i][j][0] << "," << map[i][j][1] << "," << map[i][j][2] << "],";
            }
            cout << 'r';
        }
    }

};


struct Node {
    static vector<Node*> nodes_lst; // List of all created nodes

    vector<int> position;
    vector<Node*> neighbours;
    Node* parent;
    float g_cost;
    float h_cost;
    Graph* graph;
    
    Node( vector<int> position_v, Graph* graph_v, float g_cost_v = 0, float h_cost_v = 0){
        position = position_v;
        neighbours = {};
        parent = nullptr;
        g_cost = g_cost_v;
        h_cost = h_cost_v;
        graph = graph_v;

        Node::nodes_lst.push_back(this);
    }

    void getNeighbours() {
        for (int i = 0; i <= 2; i++) {
            for (int j = 0; j <= 2; j++) {
                int newX = position[0] + i - 1;
                int newY = position[1] + j - 1;
                // check if the neighbouring node to be created is inside the map boundary
                if (newX >= 0 && newX <= graph->map_dim-1 && newY >= 0 && newY <= graph->map_dim-1) {
                    // check if the neighbouring node already exists
                    bool nodeAlreadyExists = false;
                    for (Node* node : Node::nodes_lst) {
                        if (node->position[0] == newX && node->position[1] == newY) {
                            nodeAlreadyExists = true;
                            if (i!=1 && j!=1){ // neighbouring node is not the parent node
                                neighbours.push_back(node);
                            }
                            break;
                        }
                    }
                    // only consider creating a node if it does not exist
                    if (nodeAlreadyExists == false) {
                        vector<float> zero_vec = {0.0, 0.0, 0.0};
                        // checks if the considered neighbouring node position is same as its parent position or is an obstacle
                        if ((i == 1 && j == 1) || graph->map[newX][newY] == zero_vec ) {
                            continue;
                        } else {
                            Node* neigh = new Node( {newX , newY}, graph, 1000, 1000);
                            neighbours.push_back(neigh);
                        }
                    }
                }
            }
        }
    }

    string toString() {
        return "Node [" + to_string(position[0]) + ", " + to_string(position[1]) + "]";
    }

};
vector<Node*> Node::nodes_lst = {};



pair< vector<Node*> , vector<Node*> > Astar(Graph* graph){

    // Define a lambda function to compute the distance between two nodes
    auto dist = [](vector<int> lst1, vector<int> lst2) {
        double distance = sqrt(pow(lst1[0] - lst2[0], 2) + pow(lst1[1] - lst2[1], 2));
        return distance;
    };

    vector<Node*> closed_list;
    vector<Node*> open_list = { new Node(graph->initial_position, graph) };

    float least_cost;
    float cost;
    Node* current_node;

    vector<Node*> path = {};

    while(open_list.size()){ // while open list is not empty
        
        // Take from the open list the node current_node with the highest gain
        least_cost = INT_MAX;
        for(auto i : open_list){
            cost = i->g_cost + i->h_cost;

            if (cost < least_cost){
                least_cost = cost; 
                current_node = i; // set as current node
            }
        }

        // if current_node is the goal node, extract and return path
        if (current_node->position == graph->goal_position){
            vector<Node*> path = {current_node};

            while(current_node->parent != nullptr){
                path.insert(path.begin(), current_node->parent);
                current_node = current_node->parent;
            }
            return {path , closed_list};        
        }
        else{
            if ((current_node->neighbours).size() == 0){
                current_node->getNeighbours();
            }

            for (auto i : current_node->neighbours){
                float neighbour_cost = current_node->g_cost + dist(current_node->position,i->position);

                if (find(open_list.begin(), open_list.end(), i) != open_list.end()){
                    if (i->g_cost <= neighbour_cost){
                        // do nothing
                    }
                    else{
                        i->g_cost = neighbour_cost;
                        i->parent = current_node;
                    }
                }
                else if(find(closed_list.begin(), closed_list.end(), i) != closed_list.end()){
                    if (i->g_cost <= neighbour_cost){
                        // do nothing
                    }
                    else{
                        auto it = find(closed_list.begin(), closed_list.end(), i);
                        int ind = distance(closed_list.begin(), it);
                        auto to_be_added_to_open = closed_list[ind];
                        closed_list.erase(closed_list.begin() + ind);
                        if (find(open_list.begin(), open_list.end(), to_be_added_to_open) == open_list.end()) {
                            open_list.push_back(to_be_added_to_open);
                        }
                        i->g_cost = neighbour_cost;
                        i->parent = current_node;
                    }
                }
                else{
                    if( find(open_list.begin(), open_list.end(), i) == open_list.end() ){
                        open_list.push_back(i);
                    }
                    i->h_cost = dist(i->position , graph->goal_position);
                    i->g_cost = neighbour_cost;
                    i->parent = current_node;
                }
            }

            auto it = find(open_list.begin(), open_list.end(), current_node);
            int ind = distance(open_list.begin(), it);
            auto to_be_added_to_closed = open_list[ind];
            open_list.erase(open_list.begin() + ind);
            if (find(closed_list.begin(), closed_list.end(), to_be_added_to_closed) == closed_list.end()) {
                closed_list.push_back(to_be_added_to_closed);
            }
        }
    }
    return {path , closed_list};
}






int main() {
    int map_dim = 40;           // Change the dimension of the map as needed
    float obstacle_intensity = 0.05; // Change the obstacle intensity as needed
    
    Graph My_Graph = Graph(map_dim, obstacle_intensity); // Create graph

    pair< vector<Node*> , vector<Node*> > sol = Astar(&My_Graph); // Solve for the path
    vector<Node*> solpath = sol.first;
    vector<Node*> solclosed = sol.second;

    // Get the path as a vector if paris of integers rather than a vector of node pointers
    pair<int,int> cur_node_position;

    vector<pair<int,int>> solpathpair;
    for (auto i : solpath){
        cur_node_position = {i->position[0],i->position[1]};
        solpathpair.push_back(cur_node_position);
        }

    vector<pair<int,int>> visitednodespair;
    for (auto i : solclosed){
        cur_node_position = {i->position[0],i->position[1]};
        visitednodespair.push_back(cur_node_position);
        }

    My_Graph.color_and_print_map(solpathpair , visitednodespair);

    return 0;
}