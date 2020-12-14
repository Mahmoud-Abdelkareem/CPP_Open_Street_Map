#include "route_planner.h"
#include <algorithm>

RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // TODO 2: Use the m_Model.FindClosestNode method to find the closest nodes to the starting and ending coordinates.
    // Store the nodes you find in the RoutePlanner's start_node and end_node attributes.
	start_node = &m_Model.FindClosestNode(start_x,start_y);
  	end_node = &m_Model.FindClosestNode(end_x,end_y);
}


// TODO 3: Implement the CalculateHValue method.
// Tips:
// - You can use the distance to the end_node for the h value.
// - Node objects have a distance method to determine the distance to another node.

float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {

  return node->distance(*end_node); 
  //this will return std::sqrt(std::pow((x - other.x), 2) + std::pow((y - other.y), 2));
}


// TODO 4: Complete the AddNeighbors method to expand the current node by adding all unvisited neighbors to the open list.
// Tips:
// - Use the FindNeighbors() method of the current_node to populate current_node.neighbors vector with all the neighbors.
// - For each node in current_node.neighbors, set the parent, the h_value, the g_value. 
// - Use CalculateHValue below to implement the h-Value calculation.
// - For each node in current_node.neighbors, add the neighbor to open_list and set the node's visited attribute to true.

void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
  
  current_node->FindNeighbors(); //this will add the nodes in the neighbors vector attribute
  for (auto neighbor : current_node->neighbors){
    neighbor->parent = current_node;//set current node as the parent
    neighbor->visited = true; // set node's visited attribute to true
    neighbor->g_value = current_node->g_value + current_node->distance(*neighbor); 
    // using current g value of the node plus the distance to the neighbor we are iterating in the vector
    neighbor->h_value = CalculateHValue(neighbor); //calculating h value of the neighbor
    open_list.push_back(neighbor); // adding the neighbor to the open list   
  }

}


// TODO 5: Complete the NextNode method to sort the open list and return the next node.
// Tips:
// - Sort the open_list according to the sum of the h value and g value.
// - Create a pointer to the node in the list with the lowest sum.
// - Remove that node from the open_list.
// - Return the pointer.

RouteModel::Node *RoutePlanner::NextNode() {

 // sort using a lambda expression 
    std::sort(open_list.begin(), open_list.end(), [](const auto &node1, const auto &node2) {
        return ((node1->g_value + node1->h_value) > (node2->g_value + node2->h_value));   
    }); 

  	RouteModel::Node *lowest_node = open_list.back(); //Returns a reference to the last element
  	open_list.pop_back(); // It is used to remove the last element in the vector
  
  return lowest_node;
  
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
  	RouteModel::Node last; //to store a pointer to the parent node of the current node
  while(current_node->parent != nullptr){
    path_found.push_back(*current_node); //add the current node to the path found vector
    last = *(current_node->parent);
    distance += current_node->distance(last); // adding the distance from the node to its parent
    current_node = current_node->parent; //setting the new current node to be the parent to complete the iteration
  }
  
  path_found.push_back(*current_node); // add the first node
  std::reverse(path_found.begin(),path_found.end()); //using the standard librar reverse function to rearrange the vector so the start node will be the first

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
    RouteModel::Node *current_node = nullptr;

    // TODO: Implement your solution here.
  //first we need to start with the start node and adding it to the open list
  start_node->visited = true;
  open_list.push_back(start_node);
  while(open_list.size()>0){
    current_node = NextNode(); // to get the next neighbor node with lowest g and h values 
    if(current_node->distance(*end_node) == 0){ //check if we reached the goal
     m_Model.path = ConstructFinalPath(current_node); //we reached the goal so construct the path
    }
    else {
      AddNeighbors(current_node); //keep adding neighbors
    }
  }

}