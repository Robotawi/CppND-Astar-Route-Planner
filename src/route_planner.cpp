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
    return node->distance(*end_node);
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

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



// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

 bool RoutePlanner::Compare (const RouteModel::Node * n1, const RouteModel::Node * n2){
    float f1 = n1->g_value + n1->h_value;
    float f2 = n2->g_value + n2->h_value;
    return f1<f2;
}
void RoutePlanner::NodeSort(std::vector<RouteModel::Node*> &openlist){
    //sort the openlist and return it
    //the nodes are being pointed to by the vector elements, dereferene them
    //std::cout << " Inside NodeSort \n";
    std::sort(openlist.begin(), openlist.end(), Compare);

}
RouteModel::Node* RoutePlanner::NextNode(std::vector<RouteModel::Node*> &openlist) {//ref to vector of pointers to nodes
    //receives the a pointer to openlist, checks the nodes and returns a pointer to the node of lowest F value as the next node to check
    //pointer to vector of nodes
    // TODO: Note that openlist is originally a member in the Routeplanner
    //std::cout << " Inside NextNode \n";
    NodeSort(openlist);
  	//uncomment to check the expected processing is happeninig
    //std::cout << " Openlist size is: "<< openlist.size()<<" \n";
    //std::cout << " After NodeSort \n";
    //auto F0 = openlist[0]->g_value+openlist[0]->h_value;
    //std::cout << "F0 value is "<< F0<<"\n";
    //auto Fe = openlist.back()->g_value+openlist.back()->h_value;
    //std::cout << "Fe value is "<< Fe<<"\n";
    RouteModel::Node* nextnode = openlist.front();
    openlist.erase(openlist.begin());
    //std::cout << " After erase Openlist size is: "<< openlist.size()<<" \n";
    return nextnode;
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
    //int count = 0;
    distance = 0.0f;
    std::vector<RouteModel::Node> path_found;
    std::cout<<"inisde ConstructFinalPath \n";

    //if the current node is not the start (has a parent), add the node and add the distance
    //if the start node is reached, add the node and not the distance

    //one way to do it (has a problem)

    while(current_node->parent != nullptr){
        //count++;
        //std::cout<<"The final path is being built..."<<count<<" \n";
        path_found.push_back(*current_node);
        distance += current_node->distance(*(current_node->parent));
        current_node = current_node->parent;
    }

    //another way to do it
    /*
    while(not(current_node->x == start_node->x && start_node->y == start_node->y)){
        path_found.push_back(*current_node);
        distance += current_node->distance(*(current_node->parent));
        current_node = current_node->parent;
    }
     */
    //insert the start node
    path_found.push_back(*current_node);
    std::cout<< "path size is "<<path_found.size()<<" \n";
    std::reverse(path_found.begin(), path_found.end());
    distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
  	std::cout<<"Path found and returned! \n";
    return path_found;
}


// TODO 7: Write the A* Search algorithm here.
// Tips:
// - Use the AddNeighbors method to add all of the neighbors of the current node to the open_list.
// - Use the NextNode() method to sort the open_list and return the next node.
// - When the search has reached the end_node, use the ConstructFinalPath method to return the final path that was found.
// - Store the final path in the m_Model.path attribute before the method exits. This path will then be displayed on the map tile.

void RoutePlanner::AStarSearch() {
    //std::cout << " Inside Astarsearch \n";
    //while processing, think that you are dealing with a current node
    RouteModel::Node * current_node = start_node;
    //current_node->parent = nullptr;
    current_node->visited = true;
    current_node->g_value = 0;
    current_node->h_value = CalculateHValue(current_node);
    open_list.push_back(current_node);

    while (!open_list.empty()){
        //make sure the current node is decided out before the if/else check!
        current_node = NextNode(open_list);

        if (current_node->x == end_node->x && current_node->y == end_node->y){
            std::cout <<"Goal node found! \n";
            //search is done, and path is found
            m_Model.path = ConstructFinalPath(current_node);
            std::cout<<"***To check the found path order*** \n";
            std::cout <<"User: Start: x = "<< start_node->x << ", y = "<< start_node->y << " \n";
            std::cout <<"User: End: x = "<< end_node->x << ", y = "<< end_node->y << " \n";
            std::cout <<"Final path constructed! \n";
            std::cout <<"Final path: Start: x = "<< m_Model.path.front().x << ", y = "<< m_Model.path.front().y << " \n";
            std::cout <<"Final path: End: x = "<< m_Model.path.back().x << ", y = "<< m_Model.path.back().y << " \n";
            return;
        }
        else{
            std::cout <<"Not goal node, expanding neighbors... \n";
            AddNeighbors(current_node);
        }
    }
}