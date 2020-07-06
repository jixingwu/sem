#ifndef TOPOMETRIC_H
#define TOPOMETRIC_H

#include <iostream>
#include <fstream>

#include <Eigen/Geometry>
#include <Eigen/Dense>

#include <boost/tokenizer.hpp> 
#include <boost/lexical_cast.hpp> 

using namespace std;

class Node_c
{
public:
    Node_c(float _x, float _y, float _z, int _id_self);
    ~Node_c();

    // set coordinate and id of node
    void setxyzid(float _x, float _y, float _z, int _id_self);

    // add edge with other node with _id_neibor
    void addLink(int _id_neibor, Node_c* _node_neibor, float _dist);

    // remove edge with other node
    void removeLink(int _id_neibor);

    // coordinate of this node
    Eigen::Vector3d xyz;
    double norm;
    int id_self;
//private:
    //edge between this node and neibor node
    vector<int> id_neibor;

    //store pointer of neibor node, coresponding to id_neibor
    vector<Node_c*> node_neibor;

    //store distance of neibor, coresponding to id_neibor
    vector<float> dist_neibor;

    //store whether this direction of edge is visited
    vector<bool> visited_neibor;
};

class TopoMetric_c
{
public:
    TopoMetric_c(string _frame_id);
    ~TopoMetric_c(){}

    // read graph from filepath folder, files named as node.txt and edge.txt
    bool readAndContructTopoMetric(const string& filepath);

    // return number of nodes
    int num_of_nodes(){return nodes_vec.size();}

    // return node with fake iterator _i(not same with id, but if you did not insert or delete any nodes, they are the same)
    Node_c* getNode(int _i){return &(nodes_vec[_i]);}

    // search the nearest node to input
    int searchNearestNode(Eigen::Vector3d _xyz);

    // search nearby node
    int searchNearbyNode(Eigen::Vector3d _xyz, int id_initnode);
//private:
    // nodes of the graph
    vector<Node_c> nodes_vec;

    // frame id
    string frame_id;

    // range of  nodes
    float x_upb, x_lowb, y_upb, y_lowb, z_upb, z_lowb;
};



#endif
