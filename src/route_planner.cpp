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


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    // h is the distance between the current and the end
    // this function receives the node, then we need to calculate the H value.
    // TODO: my recheck, is only the distance enough?
    float h = node->distance(*end_node);
    return h;
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (auto neighbor : current_node->neighbors){
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);//this will calculate the distance from the neighbor node to the end
        neighbor->g_value = current_node->g_value + current_node->distance(*neighbor); //*neighbor is a pointer to node content (node)
        open_list.push_back(neighbor);

    }
}



// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

 bool RoutePlanner::Compare (const RouteModel::Node & n1, const RouteModel::Node & n2){
    float f1 = n1.g_value + CalculateHValue(&n1);
    float f2 = n2.g_value + CalculateHValue(&n2);
    return f1>f2;
}
void RoutePlanner::NodeSort(std::vector<RouteModel::Node*> &openlist){
    //sort the openlist and return it
    //the nodes are being pointed to by the vector elements, dereferene them
    std::sort(*openlist.begin(), *openlist.end(), &RoutePlanner::Compare);

}
RouteModel::Node* RoutePlanner::NextNode(std::vector<RouteModel::Node*> &openlist) {//ref to vector of pointers to nodes
    //receives the a pointer to openlist, checks the nodes and returns a pointer to the node of lowest F value as the next node to check
    //pointer to vector of nodes
    // TODO: Note that openlist is originally a member in the Routeplanner
    NodeSort(openlist);
    return openlist[0];
    }


// TODO 6: Complete the ConstructFinalPath method to return the final path found from your A* search.
// Tips:
// - This method should take the current (final) node as an argument and iteratively follow the 
//   chain of parents of nodes until the starting node is found.
// - For each node in the chain, add the distance from the node to its parent to the distance variable.
// - The returned vector should be in the correct order: the start node should be the first element
//   of the vector, the end node should be the last element.

std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
    // Create path_found vector
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;

    // TODO: Implement your solution here.

    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
    return path_found;

}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    //takes start, end,
    RouteModel::Node *current_node = nullptr;
    //its astar search will search its own private variable?
    // TODO: Implement your solution here.
    AddNeighbors(start_node);
    NextNode(open_list);




}