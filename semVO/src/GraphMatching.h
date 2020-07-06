
/// p_prior = q_prior_visited * p_visited + t_prior_visited

#ifndef GRAPHMATCHING_H
#define GRAPHMATCHING_H

#include "TopoMetric.h"

#include <iostream>
#include <fstream>
#include <map>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <unsupported/Eigen/CXX11/Tensor>

#include <opencv2/core/core.hpp>

using namespace std;

template <typename GraphT, typename NodeT>
class GraphMatching
{
public:
    GraphMatching();
    ~GraphMatching(){}
    //read parameter
    void readParam(string paramfile);

    //init method before any match
    // in this function, tm1 is prior graph, tm2 is slam graph
    void initialize(GraphT* tm1);

    // match tm1 and tm1 to find the similar part
    void matchSelf();

    // calculate the matching of two topometric graph tm1 and tm2 using vote tensor
    void matchTwoTopoMetric(GraphT* tm2);

    // calculate the matching of two topometric graph tm1 and tm2 assuming one match
    void matchTwoTopoMetricWithOneMatch(GraphT *tm2);

    // get probability of current state(at which node)
    vector<double> getProbOfAllStates(){return prob_vec;}

    // input: id of a node in prior graph
    // return: corresponding xyz position in visited frame with the best match
    Eigen::Vector3d getCoordInVisitedFromIDPrior(int id_prior);

    // should be used after matching
    int getIDPriorFromIDVisited(int id_visited);

    // just for testing some function
    void test();

    void inputSimMatrix(Eigen::MatrixXd& sim_mat, Eigen::MatrixXd& sim_results);
private:
    // calculate affinity of two nodes
//    double calAffinityOfTwoNode(NodeT* nc1, NodeT* nc2);

    // clear some storage
    void clear();

    //
    void calculateVoteTensor();

    // calculate similarity with q and t
    // p1 = q * p2 + t
    double calSimilarityOfTwoNode(NodeT* nc1, NodeT* nc2, Eigen::Quaterniond q_1_2, Eigen::Vector3d t_1_2);

    // calculate similarity with only node information
    double calSimilarityOfTwoNode(NodeT* nc1, NodeT* nc2);

    // calculate distance of two points
    double distanceOfTwoPoint(Eigen::Vector3d p1, Eigen::Vector3d p2);

    // get best match
    // return a new err/sim matrix which error match is removed
    Eigen::MatrixXd getBestMatchFromErrMat(Eigen::MatrixXd err_mat, Eigen::VectorXi& retmatch);
    Eigen::MatrixXd getBestMatchFromSimMat(Eigen::MatrixXd sim_mat, Eigen::VectorXi& retmatch, Eigen::VectorXi& retmatchinverse);

    // sinkhorn
    Eigen::MatrixXd sinkhorn(Eigen::MatrixXd sim_mat);

    // get best n result
    void getBestResultFromVoteTensor(Eigen::Tensor<double, 3> _vote_tensor, int topN, vector<Eigen::Vector3i>& _index_of_votetensor);

    //turn int a,b,c into a_b_c all abc are with 5 string number
    string turntoString(int a, int b, int c);
    //turn a,b,c into t_prior_visited
    Eigen::Vector3d turntot(int a, int b, int c);

    // show result
    void showResult(vector<Eigen::Vector3i> _index_of_votetensor);

    //graphs
    GraphT *prior_graph, *visited_graph;

    //vote related
    //tensor for storing vote value
    Eigen::Tensor<double, 3> vote_tensor;
    //a storage for matched pairs in this vote, '' is from prior to visited, 'inverse' is from visited to prior
    map<string, Eigen::VectorXi> vote_match_map;
    map<string, Eigen::VectorXi> vote_match_inverse_map;
    //a storage for q_prior_visited and t_prior_visited in this vote
    map<string, Eigen::Quaterniond> vote_qpv_map;
//    map<string, Eigen::Vector3d> vote_tpv_map;
    //
    int voteT_len_x, voteT_len_y, voteT_len_z;
    // store index of best match results in vote tensor
    vector<Eigen::Vector3i> index_of_votetensor;

    // store affinity matrix with size N(tm1)*N(tm2), N() is the number of nodes
    Eigen::MatrixXd affinity_mat;

    // map from probability vector to states
    map<int, string> probid_state_map;
    map<string, int> state_probid_map;

    // probability of current state
    vector<double> prob_vec;

    // parameters
    double vote_resolution;
    double vote_threshold;
    double scale_factor;
    double dist_factor;
    double max_sinkhorn_iter;
};


template <typename GraphT, typename NodeT>
GraphMatching<GraphT, NodeT>::GraphMatching(){}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::readParam(string paramfile)
{
    cv::FileStorage fsSettings(paramfile.c_str(), cv::FileStorage::READ);
    fsSettings["ampt"]["graphmatching"]["vote_resolution"] >> vote_resolution;
    fsSettings["ampt"]["graphmatching"]["vote_threshold"] >> vote_threshold;
    fsSettings["ampt"]["graphmatching"]["scale_factor"] >> scale_factor;
    fsSettings["ampt"]["graphmatching"]["dist_factor"] >> dist_factor;
    fsSettings["ampt"]["graphmatching"]["max_sinkhorn_iter"] >> max_sinkhorn_iter;
    cout<<"graph matching param: "<<vote_resolution<<" "<<vote_threshold<<" "<<scale_factor<<" "<<dist_factor<<endl;
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::initialize(GraphT* tm1)
{
    prior_graph = tm1;
    voteT_len_x = int((prior_graph->x_upb - prior_graph->x_lowb) / vote_resolution)+1;
    voteT_len_y = int((prior_graph->y_upb - prior_graph->y_lowb) / vote_resolution)+1;
    voteT_len_z = int((prior_graph->z_upb - prior_graph->z_lowb) / vote_resolution)+1;
    vote_tensor = Eigen::Tensor<double, 3>(voteT_len_x, voteT_len_y, voteT_len_z);
    vote_tensor.setZero();
    cout<<"vote tensor size: "<<voteT_len_x<<", "<<voteT_len_y<<", "<<voteT_len_z<<endl;
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::clear()
{
    vote_match_map.clear();
    vote_qpv_map.clear();
    index_of_votetensor.clear();
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::matchTwoTopoMetric(GraphT* tm2)
{
    visited_graph = tm2;
    clear();
    calculateVoteTensor();

    //get posible votes
    //test: get best n result
    getBestResultFromVoteTensor(vote_tensor, 5, index_of_votetensor);
    showResult(index_of_votetensor);
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::matchSelf()
{
    visited_graph = prior_graph;
    clear();
    calculateVoteTensor();

    getBestResultFromVoteTensor(vote_tensor, -1, index_of_votetensor);
    showResult(index_of_votetensor);
    // (todo: deal with the result for active localization)
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::calculateVoteTensor()
{
    Eigen::Vector3d t_prior_visited, p_prior, p_visited;
    Eigen::Vector3d p_visited_tran;
    Eigen::Quaterniond q_prior_visited, q_prior_visited_fix;
    //to handle nodes of visited graph near zero
    vector<int> nodezero_vec;
    bool record_nodezero_flag = false;
    double best_sim_score;
    for(int i_x=0; i_x<voteT_len_x; ++i_x)
    {
        t_prior_visited(0) =  vote_resolution * i_x + prior_graph->x_lowb;
        for(int i_y=0; i_y<voteT_len_y; ++i_y)
        {
            t_prior_visited(1) =  vote_resolution * i_y + prior_graph->y_lowb;
            for(int i_z=0; i_z<voteT_len_z; ++i_z)
            {
                cout<<"-------------------------------\\"<<endl<<"vote grid:"<<i_x<<", "<<i_y<<", "<<i_z<<endl;
                Eigen::MatrixXd tmpSim_mat = Eigen::MatrixXd::Zero(prior_graph->num_of_nodes(), visited_graph->num_of_nodes());
                t_prior_visited(2) =  vote_resolution * i_z + prior_graph->z_lowb;
                string keystr = turntoString(i_x, i_y, i_z);
                best_sim_score = 0;
                for(int i_pri=0; i_pri<prior_graph->num_of_nodes(); ++i_pri)
                {
                    p_prior = prior_graph->getNode(i_pri)->xyz - t_prior_visited;
                    for(int i_visi=0; i_visi<visited_graph->num_of_nodes(); ++i_visi)
                    {
                        if(visited_graph->getNode(i_visi)->norm > 1){
                            p_visited = visited_graph->getNode(i_visi)->xyz;
                            q_prior_visited = Eigen::Quaterniond::FromTwoVectors(p_visited, p_prior);
                            p_visited_tran = q_prior_visited * p_visited;
                            float center_dist = (p_prior - p_visited_tran).norm();
                            if(center_dist < vote_resolution/2){
                                cout<<"id_pri:"<<i_pri<<", id_visi:"<<i_visi<<", centerdist:"<<center_dist
                                    <<", q:"<<q_prior_visited.w()<<","<<q_prior_visited.x()<<","
                                    <<q_prior_visited.y()<<","<<q_prior_visited.z()
    //                               <<", t:"<<t_prior_visited<<", q:"<<q_prior_visited.toRotationMatrix()
    //                               <<", p_prior:"<<p_prior<<", p_visited_tran:"<<p_visited_tran
                                   <<endl;
                                double simscore = calSimilarityOfTwoNode(
                                            prior_graph->getNode(i_pri), visited_graph->getNode(i_visi)
                                          , q_prior_visited, t_prior_visited);
                                if(simscore > vote_threshold && simscore > tmpSim_mat(i_pri, i_visi)){
                                    tmpSim_mat(i_pri, i_visi) = simscore;
                                    if(simscore > best_sim_score){
                                        q_prior_visited_fix = q_prior_visited;
                                        best_sim_score = simscore;
                                    }
                                }
                            }
                        }
                        else if(!record_nodezero_flag){
                            //p_visited is near 0, only record when first see this node
                            nodezero_vec.push_back(i_visi);
                        }
                    }
                    record_nodezero_flag = true;
                }
                /// a hiding bug: q_prior_visited_fix might be wrong(to improve)
                //deal with nodes near zero
                for(int i_pri=0; i_pri<prior_graph->num_of_nodes(); ++i_pri)
                {
                    p_prior = prior_graph->getNode(i_pri)->xyz - t_prior_visited;
                    for(int i_vec=0; i_vec<nodezero_vec.size(); ++i_vec)
                    {
                        int i_visi = nodezero_vec[i_vec];
                        p_visited = visited_graph->getNode(i_visi)->xyz;
                        p_visited_tran = q_prior_visited_fix * p_visited;
                        float center_dist = (p_prior - p_visited_tran).norm();
                        if(center_dist < vote_resolution/2){
                            cout<<"id_pri:"<<i_pri<<", id_visi:"<<i_visi<<", centerdist:"<<center_dist
                                <<", q:"<<q_prior_visited_fix.w()<<","<<q_prior_visited_fix.x()<<","
                                <<q_prior_visited_fix.y()<<","<<q_prior_visited_fix.z()
                                <<endl;
                            double simscore = calSimilarityOfTwoNode(
                                        prior_graph->getNode(i_pri), visited_graph->getNode(i_visi)
                                      , q_prior_visited_fix, t_prior_visited);
                            if(simscore > vote_threshold && simscore > tmpSim_mat(i_pri, i_visi)){
                                tmpSim_mat(i_pri, i_visi) = simscore;
                            }
                        }
                    }
                }
                Eigen::VectorXi retmatch(prior_graph->num_of_nodes())
                        , retmatchinverse(visited_graph->num_of_nodes());
                retmatch.setConstant(-1);
                retmatchinverse.setConstant(-1);
                vote_tensor(i_x, i_y, i_z) = getBestMatchFromSimMat(tmpSim_mat, retmatch, retmatchinverse).sum();
                vote_match_map[keystr] = retmatch;
                vote_match_inverse_map[keystr] = retmatchinverse;
                vote_qpv_map[keystr] = q_prior_visited_fix;
                cout<<"vote_tensor:"<<vote_tensor(i_x, i_y, i_z)<<endl;
                for(int tmpi=0; tmpi<prior_graph->num_of_nodes(); ++tmpi){
                    cout<<tmpi<<"~"<<retmatch(tmpi)<<"==";
                }
                cout<<"----------------------/"<<endl;
                record_nodezero_flag = true;
            }
        }
    }
}

template <typename GraphT, typename NodeT>
double GraphMatching<GraphT, NodeT>::calSimilarityOfTwoNode(NodeT* nc1, NodeT* nc2, Eigen::Quaterniond q_1_2, Eigen::Vector3d t_1_2)
{
    if(nc1->id_neibor.size() == 0){
        return 0.0;
    }
    double ret = 0.0;
    vector<Eigen::Vector3d> xyz_neibor_2_vec;
    xyz_neibor_2_vec.resize(nc2->id_neibor.size());
    //transform nc2 points to nc1 frame
    for(int i_2=0; i_2<nc2->id_neibor.size(); ++i_2)
    {
        xyz_neibor_2_vec[i_2] = q_1_2 * nc2->node_neibor[i_2]->xyz + t_1_2;
    }
    //calculate error of any two neibor nodes of two nodes
    Eigen::MatrixXd err_mat;
    err_mat.resize(nc1->id_neibor.size(), nc2->id_neibor.size());
    for(int i_1=0; i_1<nc1->id_neibor.size(); ++i_1)
    {
        for(int i_2=0; i_2<nc2->id_neibor.size(); ++i_2)
        {
            err_mat(i_1, i_2) = distanceOfTwoPoint(nc1->node_neibor[i_1]->xyz, xyz_neibor_2_vec[i_2]);
        }
    }
//    cout<<"err mat:"<<endl<<err_mat<<endl;
    //get most near pairs
    Eigen::VectorXi retmatch(nc1->id_neibor.size());
    retmatch.setConstant(-1);
    getBestMatchFromErrMat(err_mat, retmatch); ///maybe here can use similarity mat with sinkhorn
    //calculate similarity
    for(int i=0; i<retmatch.size(); ++i)
    {
        if(retmatch[i] == -1){
            continue;
        }
        else{
            ret += exp(-err_mat(i, retmatch[i]));
            /// maybe gausian model is better?
        }
    }
    ret = ret / nc1->id_neibor.size();
    cout<<"similarity between G1 node "<<nc1->id_self<<" and G2 node "<<nc2->id_self
       <<" is:"<<ret<<endl;
    return ret;
}

template <typename GraphT, typename NodeT>
double GraphMatching<GraphT, NodeT>::calSimilarityOfTwoNode(NodeT* nc1, NodeT* nc2)
{}

template <typename GraphT, typename NodeT>
Eigen::MatrixXd GraphMatching<GraphT, NodeT>::getBestMatchFromErrMat(Eigen::MatrixXd err_mat, Eigen::VectorXi& retmatch)
{
    Eigen::MatrixXd::Index minRow, minCol;
    Eigen::MatrixXd new_mat = Eigen::MatrixXd::Zero(err_mat.rows(), err_mat.cols());
    Eigen::VectorXd colmax = Eigen::VectorXd::Constant(err_mat.rows(), DBL_MAX)
            , rowmax = Eigen::VectorXd::Constant(err_mat.cols(), DBL_MAX);
    double minvalue;
    for(int row=0; row<err_mat.rows(); ++row)
    {
        minvalue = err_mat.minCoeff(&minRow, &minCol);
        if(minvalue > 100000000){break;}
        retmatch(minRow) = minCol;
//        cout<<"row:"<<minRow<<", col:"<<minCol<<endl;
        // set the value of matched nodes to max
        new_mat(minRow, minCol) = minvalue;
        err_mat.col(minCol) = colmax;
        err_mat.row(minRow) = rowmax;
    }

    return new_mat;
}


template <typename GraphT, typename NodeT>
Eigen::MatrixXd GraphMatching<GraphT, NodeT>::getBestMatchFromSimMat(Eigen::MatrixXd sim_mat, Eigen::VectorXi& retmatch, Eigen::VectorXi& retmatchinverse)
{
    Eigen::MatrixXd::Index maxRow, maxCol;
    Eigen::MatrixXd new_mat = Eigen::MatrixXd::Zero(sim_mat.rows(), sim_mat.cols());
    Eigen::VectorXd colmin = Eigen::VectorXd::Constant(sim_mat.rows(), 0)
            , rowmin = Eigen::VectorXd::Constant(sim_mat.cols(), 0);
    double maxvalue;
//    cout<<"simmat:"<<sim_mat<<endl;
    // with sinkhorn
//    sim_mat = sinkhorn(sim_mat);
    for(int row=0; row<sim_mat.rows(); ++row)
    {
        maxvalue = sim_mat.maxCoeff(&maxRow, &maxCol);
        if(maxvalue < 0.01){break;}
        retmatch(maxRow) = maxCol;
        retmatchinverse(maxCol) = maxRow;
        cout<<"row:"<<maxRow<<", col:"<<maxCol<<endl;
        // set the value of matched nodes to max
        new_mat(maxRow, maxCol) = maxvalue;
        sim_mat.col(maxCol) = colmin;
        sim_mat.row(maxRow) = rowmin;
    }

    return new_mat;
}

template <typename GraphT, typename NodeT>
Eigen::MatrixXd GraphMatching<GraphT, NodeT>::sinkhorn(Eigen::MatrixXd sim_mat)
{
    double sum_of_rc;
    for(int iter=0; iter<max_sinkhorn_iter; ++iter)
    {
        for(int row=0; row<sim_mat.rows(); ++row)
        {
            sum_of_rc = sim_mat.row(row).sum();
            if(sum_of_rc < 0.1) continue;
            sim_mat.row(row) = sim_mat.row(row) / sum_of_rc;
        }
        for(int col=0; col<sim_mat.cols(); ++col)
        {
            sum_of_rc = sim_mat.col(col).sum();
            if(sum_of_rc < 0.1) continue;
            sim_mat.col(col) = sim_mat.col(col) / sum_of_rc;
        }
    }

    cout<<"after fakesinkhorn:"<<sim_mat<<endl;
    return sim_mat;
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::getBestResultFromVoteTensor(Eigen::Tensor<double, 3> _vote_tensor, int topN, vector<Eigen::Vector3i>& _index_of_votetensor)
{
    if(topN < 0){
        topN = INT_MAX;
    }
    //
    for(int i_x=0; i_x<voteT_len_x; ++i_x)
    {
        for(int i_y=0; i_y<voteT_len_y; ++i_y)
        {
            for(int i_z=0; i_z<voteT_len_z; ++i_z)
            {
                double value = _vote_tensor(i_x, i_y, i_z);
                // judge and insert to return vector
                if(value > 0.1){
                    if(_index_of_votetensor.size() == 0){
                        //nothing in vector
                        _index_of_votetensor.push_back(Eigen::Vector3i(i_x, i_y, i_z));
                    }
                    else{
                        if(value < _vote_tensor(_index_of_votetensor[_index_of_votetensor.size()-1](0),
                               _index_of_votetensor[_index_of_votetensor.size()-1](1),
                               _index_of_votetensor[_index_of_votetensor.size()-1](2))){
                            // smaller than last one
                            if(_index_of_votetensor.size()<topN){
                                _index_of_votetensor.push_back(Eigen::Vector3i(i_x, i_y, i_z));
                            }
                            continue;
                        }
                        else if(value > _vote_tensor(_index_of_votetensor[0](0),
                                       _index_of_votetensor[0](1), _index_of_votetensor[0](2))){
                            // bigger than first one
                            _index_of_votetensor.insert(_index_of_votetensor.begin(), Eigen::Vector3i(i_x, i_y, i_z));
                        }
                        else{
                            // smaller than first and bigger than last one
                            for(int i_index=_index_of_votetensor.size()-1; i_index>=0; --i_index)
                            {
                                // from last on, if to a value bigger than self, insert after this
                                if(value <= _vote_tensor(_index_of_votetensor[i_index](0),
                                        _index_of_votetensor[i_index](1), _index_of_votetensor[i_index](2))){
                                    _index_of_votetensor.insert(_index_of_votetensor.begin()+1+i_index, Eigen::Vector3i(i_x, i_y, i_z));
                                    break;
                                }
                            }
                        }
                        if(_index_of_votetensor.size() > topN){
                            _index_of_votetensor.pop_back();
                        }
                    }
                }
            }
        }
    }
//    Eigen::MatrixXd::Index maxaxis0, maxaxis1, maxaxis2;
//    for(int i=0; i<topN; ++i)
//    {
//        maxvalue = _vote_tensor.maximum() (maxaxis0, maxaxis1, maxaxis2);
//        if(maxvalue < 0.1){break;}
//        _index_of_votetensor.push_back(Eigen::Vector3i(maxaxis0, maxaxis1, maxaxis2));
//        _vote_tensor(maxaxis0, maxaxis1, maxaxis2) = 0;
//    }
}

template <typename GraphT, typename NodeT>
double GraphMatching<GraphT, NodeT>::distanceOfTwoPoint(Eigen::Vector3d p1, Eigen::Vector3d p2)
{
    return (p1 - p2).norm();
}

template <typename GraphT, typename NodeT>
string GraphMatching<GraphT, NodeT>::turntoString(int a, int b, int c)
{
    char tmp[16];
    sprintf(tmp, "%05d%05d%05d\n", a,b,c);
    string str = tmp;
//    cout<<"tostring: "<<str<<endl;
    return str;
}

template <typename GraphT, typename NodeT>
Eigen::Vector3d GraphMatching<GraphT, NodeT>::turntot(int a, int b, int c)
{
    Eigen::Vector3d ret(vote_resolution * a + prior_graph->x_lowb,
                        vote_resolution * b + prior_graph->y_lowb,
                        vote_resolution * b + prior_graph->z_lowb);
    return ret;
}

template <typename GraphT, typename NodeT>
Eigen::Vector3d GraphMatching<GraphT, NodeT>::getCoordInVisitedFromIDPrior(int id_prior)
{
    Eigen::Vector3i index_bestmatch = index_of_votetensor[0];
    string keystr = turntoString(index_bestmatch(0), index_bestmatch(1), index_bestmatch(2));
    Eigen::Vector3d xyz_prior = prior_graph->nodes_vec[id_prior].xyz,
            t_prior_visited = turntot(index_bestmatch(0), index_bestmatch(1), index_bestmatch(2));
    Eigen::Quaterniond q_prior_visited = vote_qpv_map[keystr];
    Eigen::Vector3d xyz_visited = q_prior_visited.toRotationMatrix().transpose() * (xyz_prior - t_prior_visited);
    return xyz_visited;
}

template <typename GraphT, typename NodeT>
int GraphMatching<GraphT, NodeT>::getIDPriorFromIDVisited(int id_visited)
{
    Eigen::Vector3i index_bestmatch = index_of_votetensor[0];
    string keystr = turntoString(index_bestmatch(0), index_bestmatch(1), index_bestmatch(2));
    int id_prior = vote_match_inverse_map[keystr](id_visited);
    //(todo: if there is no matching node)

    return id_prior;
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::test()
{
}

template <typename GraphT, typename NodeT>
void GraphMatching<GraphT, NodeT>::showResult(vector<Eigen::Vector3i> _index_of_votetensor)
{
    // cout top n match
    string keystr;
    cout<<"show best match~~~~~~~~~~~~~~~~~~~~"<<endl;
    for(int i=0; i<_index_of_votetensor.size(); ++i)
    {
        keystr = turntoString(_index_of_votetensor[i](0), _index_of_votetensor[i](1), _index_of_votetensor[i](2));
        Eigen::VectorXi retmatch = vote_match_map[keystr];
        cout<<"------top match "<<i<<endl
           <<"index of vote tensor: "<<_index_of_votetensor[i](0)<<","<<_index_of_votetensor[i](1)<<","<<_index_of_votetensor[i](2)<<endl
           <<"t:"<<(vote_resolution * _index_of_votetensor[i](0) + prior_graph->x_lowb)<<", "
                 <<(vote_resolution * _index_of_votetensor[i](1) + prior_graph->y_lowb)<<", "
                 <<(vote_resolution * _index_of_votetensor[i](2) + prior_graph->z_lowb)<<endl
           <<"score: "<<vote_tensor(_index_of_votetensor[i](0), _index_of_votetensor[i](1), _index_of_votetensor[i](2))<<endl;
        for(int tmpi=0; tmpi<prior_graph->num_of_nodes(); ++tmpi){
            if(retmatch(tmpi) < 0) continue;
            cout<<tmpi<<"~"<<retmatch(tmpi)<<"==";
        }
        cout<<endl;
    }
}

//template <typename GraphT, typename NodeT>
//double GraphMatching<GraphT, NodeT>::calAffinityOfTwoNode(NodeT* nc1, NodeT* nc2)
//{}





#endif
