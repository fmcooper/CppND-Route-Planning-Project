#include "route_planner.h"
#include <algorithm>
#include <iostream>

using std::cout;


RoutePlanner::RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y): m_Model(model) {
    // Convert inputs to percentage:
    start_x *= 0.01;
    start_y *= 0.01;
    end_x *= 0.01;
    end_y *= 0.01;

    // Finds and saves the closest nodes to the start and end coordinates.
    start_node = & m_Model.FindClosestNode(start_x, start_y);
    end_node = & m_Model.FindClosestNode(end_x, end_y);
}


// Calculates and returns the H value for a given node.
float RoutePlanner::CalculateHValue(RouteModel::Node const *node) {
    float distance = node->distance(*end_node);
    return distance;
}


// AddNeighbors method initialises and adds all neighbors of the given node that have not 
// already been visited to the open_list.
void RoutePlanner::AddNeighbors(RouteModel::Node *current_node) {
    current_node->FindNeighbors();
    for (RouteModel::Node *neighbor : current_node->neighbors) {
        neighbor->parent = current_node;
        neighbor->h_value = CalculateHValue(neighbor);
        neighbor->g_value = current_node->g_value + neighbor->distance(*current_node);
        neighbor->visited = true;
        open_list.emplace_back(neighbor);
    }

}


// Comparison function for sorting open_list in non-increasing order.
bool NodeComparison(RouteModel::Node* a, RouteModel::Node* b) { 
  return ((a->g_value + a->h_value) > (b->g_value + b->h_value)); 
}

// NextNode method removes and returns a pointer to the smallest element from open_list.
RouteModel::Node *RoutePlanner::NextNode() {
  // Sort the vector in non-increasing order to take advantage of O(1) removal of
  // element from back of vector.
  std::sort (open_list.begin(), open_list.end(), NodeComparison); 
  
  RouteModel::Node* last_elem = open_list.back();
  open_list.pop_back();
  return last_elem; 
}


// ConstructFinalPath method returns a list of nodes indicating the final path from the 
// start node to the end node.
std::vector<RouteModel::Node> RoutePlanner::ConstructFinalPath(RouteModel::Node *current_node) {
  distance = 0.0f;
  std::vector<RouteModel::Node> path_found;
  
  // Trace back through the parent node pointers to find the path.
  bool finished = false;
  while (not finished) {
    path_found.push_back(*current_node);
    RouteModel::Node* parent_pointer = current_node->parent;
    distance = distance + current_node->distance(*parent_pointer);
    current_node = current_node->parent;
    
    // Finish iterating if the current node is the start node.
    if (current_node->parent == nullptr) {
      path_found.push_back(*current_node);
      finished = true;
    }
  }
  
  // Reverse the path so that the start node is at the start.
  std::reverse(path_found.begin(), path_found.end());

  distance *= m_Model.MetricScale(); // Multiply the distance by the scale of the map to get meters.
  return path_found;
}


// A* search algorithm
void RoutePlanner::AStarSearch() {
  RouteModel::Node *current_node = nullptr;

  // Mark the start node as visited and add the start nodes neighbors to open_list.
  start_node->visited = true;
  AddNeighbors(start_node);
  
  // Continue searching until the end node is found.
  bool finished = false;
  while (not finished) {
    current_node = NextNode();
    current_node->visited = true;
    AddNeighbors(current_node);
    
    // Finish iterating if the end node is found.
    if (current_node == end_node) {
      finished = true;
    }
  }
  
  // Construct and save the final path.
  m_Model.path = ConstructFinalPath(current_node);
}