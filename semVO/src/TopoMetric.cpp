#include "TopoMetric.h"

double calDistance(Eigen::Vector3d po1, Eigen::Vector3d po2){
    Eigen::Vector3d err = po1 - po2;
    return err.norm();
}

Node_c::Node_c(float _x, float _y, float _z, int _id_self):
    xyz(_x,_y,_z), id_self(_id_self)
{
    norm = xyz.norm();
}

Node_c::~Node_c(){}

void Node_c::setxyzid(float _x, float _y, float _z, int _id_self)
{
    xyz << _x, _y, _z;
    norm = xyz.norm();
    id_self = _id_self;
}

void Node_c::addLink(int _id_neibor, Node_c* _node_neibor, float _dist)
{
    id_neibor.push_back(_id_neibor);
    node_neibor.push_back(_node_neibor);
    visited_neibor.push_back(false);
    dist_neibor.push_back(_dist);
}

void Node_c::removeLink(int _id_neibor)
{
    for(int i=0; i<id_neibor.size(); ++i){
        if(_id_neibor == id_neibor[i]){
            id_neibor.erase(id_neibor.begin()+i);
            node_neibor.erase(node_neibor.begin()+i);
            visited_neibor.erase(visited_neibor.begin()+i);
            break;
        }
    }
}


TopoMetric_c::TopoMetric_c(string _frame_id):
    x_upb(FLT_MIN), x_lowb(FLT_MAX),
    y_upb(FLT_MIN), y_lowb(FLT_MAX),
    z_upb(FLT_MIN), z_lowb(FLT_MAX)
{
    frame_id = _frame_id;
}

bool TopoMetric_c::readAndContructTopoMetric(const string& filepath)
{
    nodes_vec.clear();
    string nodefile = filepath+"node.txt",
           edgefile = filepath+"edge.txt";

    // read nodes
    ifstream in(nodefile);
    if(!in){
        printf("[topometric]no node file!\n");
        return false;
    }
    string line;
    boost::char_separator<char> sep(" ");
    Node_c nc(0,0,0,0);
    float x,y,z;
    int i=0,j=0;
    while (!in.eof())
    {
        std::getline(in, line);
        in.peek();
        boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
        std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());
        if (tokens.size() != 3) continue;
        x = boost::lexical_cast<float>(tokens[0]);
        y = boost::lexical_cast<float>(tokens[1]);
        z = boost::lexical_cast<float>(tokens[2]);

        nc.setxyzid(x, y, z, i);
        nodes_vec.push_back(nc);
        if(x > x_upb){
            x_upb = x;
        }
        else if(x < x_lowb){
            x_lowb = x;
        }
        if(y > y_upb){
            y_upb = y;
        }
        else if(y < y_lowb){
            y_lowb = y;
        }
        if(z > z_upb){
            z_upb = z;
        }
        else if(z < z_lowb){
            z_lowb = z;
        }
//        cout<<x<<" "<<y<<" "<<z<<" "<<i<<endl;
        ++i;
    }
    in.close();

    //read edges
    ifstream in1(edgefile);
    if(!in1){
        printf("[topometric]no edge file!\n");
        return false;
    }
    while (!in1.eof())
    {
        std::getline(in1, line);
        in1.peek();
        boost::tokenizer<boost::char_separator<char> > tokenizer(line, sep);
        std::vector<std::string> tokens(tokenizer.begin(), tokenizer.end());
        if (tokens.size() != 2) continue;
        i = boost::lexical_cast<float>(tokens[0]);
        j = boost::lexical_cast<float>(tokens[1]);
        float _dist = (nodes_vec[i].xyz - nodes_vec[j].xyz).norm();

        nodes_vec[i].addLink(j, &(nodes_vec[j]), _dist);
        nodes_vec[j].addLink(i, &(nodes_vec[i]), _dist);
//        cout<<i<<" "<<j<<" "<<_dist<<endl;
    }
    in1.close();
    cout<<"[topometric]read "<<filepath<<" node and edge file done!"<<endl;

    return true;
}

int TopoMetric_c::searchNearestNode(Eigen::Vector3d _xyz)
{
    double nearest_dist = 100000, tmpdist;
    int nearest_id = -1;
    for(int id=0; id<nodes_vec.size(); ++id)
    {
        tmpdist = (nodes_vec[id].xyz - _xyz).norm();
        if(tmpdist < nearest_dist){
            nearest_dist = tmpdist;
            nearest_id = id;
        }
    }
    return nearest_id;
}

int TopoMetric_c::searchNearbyNode(Eigen::Vector3d _xyz, int id_initnode)
{
    if(id_initnode < 0) return -1;
    double nearest_dist = (nodes_vec[id_initnode].xyz - _xyz).norm(), tmpdist;
    int nearest_id = id_initnode;
    for(int neibor=0; neibor<nodes_vec[id_initnode].id_neibor.size(); ++neibor)
    {
        tmpdist = (nodes_vec[id_initnode].node_neibor[neibor]->xyz - _xyz).norm();
        // to judge whether it is reaching next node, the distance should be smaller than 1m
        if(tmpdist < 1.0 && tmpdist < nearest_dist){
            nearest_dist = tmpdist;
            nearest_id = neibor;
        }
    }
    return nearest_id;
}
