#ifndef ROUTE_PLANNER_H
#define ROUTE_PLANNER_H

#include <iostream>
#include <vector>
#include <string>
#include "route_model.h"


class RoutePlanner {
  public:
    RoutePlanner(RouteModel &model, float start_x, float start_y, float end_x, float end_y);
    // Add public variables or methods declarations here.
    float GetDistance() const {return distance;}
    void AStarSearch();

    // The following methods have been made public so we can test them individually.
    void AddNeighbors(RouteModel::Node *current_node);
    void CalculateHValue(RouteModel::Node *node);
    std::vector<RouteModel::Node> ConstructFinalPath(RouteModel::Node *);
    RouteModel::Node *NextNode(std::vector<RouteModel::Node*> &openlist); //takes the open_list private variable, passes it to NodeSort which returns a new sorted openlist, then
    //this function returns the first element in the sorted openlist

    //RouteModel::Node *NextNode(std::vector<RouteModel::Node> *);
    // to compare nodes F value
    static bool Compare (const RouteModel::Node & n1, const RouteModel::Node & n2);
    void NodeSort(std::vector<RouteModel::Node*> &openlist);

  private:
    // Add private variables or methods declarations here.
    std::vector<RouteModel::Node*> open_list; //type vector of pointers to nodes, then receive a reference to it. Then, consider that the contents are pointers, then dereference with *
    RouteModel::Node *start_node;
    RouteModel::Node *end_node;

    float distance = 0.0f;
    RouteModel &m_Model;
};

#endif