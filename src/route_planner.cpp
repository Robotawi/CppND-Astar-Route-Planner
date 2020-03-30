#include "route_planner.h"
#include <algorithm>
#include <vector>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Partial initizalization of the private variable m_Model
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
    start_node = &m_Model.FindClosestNode(start_x, start_y);
    end_node = & m_Model.FindClosestNode(end_x, end_y);
}

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // h is the distance between the current and the end
    return node->distance(*end_node);
}

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto neighbor : current_node->neighbors){ //neighbors are pointers to nodes
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);//this will calculate the distance from the neighbor node to the end
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor); //*neighbor is a pointer to node content (node)
        open_list.push_back(neighbor);
        neighbor->visited = true;
    }
}

 bool RoutePlanner::Compare (const RouteModel::Node * n1, const RouteModel::Node * n2){
    float f1 = n1->g_value + n1->h_value;
    float f2 = n2->g_value + n2->h_value;
    return f1<f2;
}
void RoutePlanner::NodeSort(std::vector<RouteModel::Node*> &openlist){
    //sort the openlist and return it
    std::sort(openlist.begin(), openlist.end(), Compare);

}
//RouteModel::Node* RoutePlanner::NextNode(std::vector<RouteModel::Node*> &openlist) {
//    //receives the a reference to openlist, checks the nodes and returns a pointer to the node of lowest F value as the next node to check
//    NodeSort(openlist);
//    RouteModel::Node* nextnode = openlist.front();
//    openlist.erase(openlist.begin());
//    return nextnode;
//    }
RouteModel::Node* RoutePlanner::NextNode() {
        std::sort(open_list.begin(), open_list.end(), [](const RouteModel::Node *a, const RouteModel::Node *b) {
            float fa = a->g_value + a->h_value;
            float fb = b->g_value + b->h_value;
            return fa > fb;
        });
        //std::cout << open_list[0]->g_value + open_list[0]->h_value << std::endl;
        //std::cout << open_list[open_list.size() - 1]->g_value + open_list[open_list.size() - 1]->h_value << std::endl;
        RouteModel::Node *next_node = open_list.back();
        open_list.pop_back();
        return next_node;
}

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    //if the current node is not the start (has a parent), add the node and add the distance
    //if the start node is reached, add the node and not the distance
    /*
    while(current_node->parent != nullptr){
        path_found.push_back(*current_node);
        distance += current_node->distance(*(current_node->parent));
        current_node = current_node->parent;
    }
    */
    //another way to do it

//    while(not(current_node->x == start_node->x && start_node->y == start_node->y)){
//        path_found.push_back(*current_node);
//        distance += current_node->distance(*current_node->parent);
//        current_node = current_node->parent;
//    }
//
//    //insert the start node
//    path_found.push_back(*current_node);
//    std::reverse(path_found.begin(), path_found.end());
//    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
//    return path_found;

    while (current_node->x != start_node->x && current_node->y != start_node->y) {
        path_found.insert(path_found.begin(), *current_node);
        distance += current_node->distance(*current_node->parent);
        current_node = current_node->parent;
    }
    path_found.insert(path_found.begin(), *current_node);
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;
}

void RoutePlanner::AStarSearch() {
    RouteModel::Node * current_node = start_node;
    //current_node->parent = nullptr;
    current_node->visited = true;
//    current_node->g_value = 0;
//    current_node->h_value = CalculateHValue(current_node);
//    open_list.push_back(current_node);
//
//    while (!open_list.empty()){
//        //make sure the current node is decided out before the if/else check!
//        current_node = NextNode(open_list);
//
//        if (current_node->x == end_node->x && current_node->y == end_node->y){
//            m_Model.path = ConstructFinalPath(current_node);
//            return;
//        }
//        else{
//            //std::cout <<"Not goal node, expanding neighbors... \n";
//            AddNeighbors(current_node);
//        }
//    }
    int i = 0;
    while(current_node != end_node) {
        AddNeighbors(current_node);
        current_node = NextNode();
        std::cout << i << ": " << "(" << current_node->x << "," << current_node->y << "): " << current_node->g_value << " + " << current_node->h_value << " = " << current_node->g_value + current_node->h_value << std::endl;
        i++;
    }
    std::cout << i << std::endl;
    m_Model.path = ConstructFinalPath(current_node);
}