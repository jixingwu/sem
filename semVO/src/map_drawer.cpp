////
//// Created by jixingwu on 2020/3/11.
////
//
//#include "map_drawer.h"
//using namespace std;
//using namespace Eigen;
//
//MapDrawer::MapDrawer()
//{
//    box_colors.push_back(Vector3f(230, 0, 0) / 255.0);	 // red  0
//    box_colors.push_back(Vector3f(60, 180, 75) / 255.0);   // green  1
//    box_colors.push_back(Vector3f(0, 0, 255) / 255.0);	 // blue  2
//    box_colors.push_back(Vector3f(255, 0, 255) / 255.0);   // Magenta  3
//    box_colors.push_back(Vector3f(255, 165, 0) / 255.0);   // orange 4
//    box_colors.push_back(Vector3f(128, 0, 128) / 255.0);   //purple 5
//    box_colors.push_back(Vector3f(0, 255, 255) / 255.0);   //cyan 6
//    box_colors.push_back(Vector3f(210, 245, 60) / 255.0);  //lime  7
//    box_colors.push_back(Vector3f(250, 190, 190) / 255.0); //pink  8
//    box_colors.push_back(Vector3f(0, 128, 128) / 255.0);   //Teal  9
//
//    all_edge_pt_ids.resize(8, 2); // draw 8 edges except front face
//    all_edge_pt_ids << 2, 3, 3, 4, 4, 1, 3, 7, 4, 8, 6, 7, 7, 8, 8, 5;
//    all_edge_pt_ids.array() -= 1;
//    front_edge_pt_ids.resize(4, 2);
//    front_edge_pt_ids << 1, 2, 2, 6, 6, 5, 5, 1;
//    front_edge_pt_ids.array() -= 1;
//}
//MapDrawer::~MapDrawer(){}
//
//void MapDrawer::DrawMapCuboids()
//{
//    const vector<MapObject *> all_Map_objs = GetAllMapObjects();
//    Vector4d front_face_color(1.0, 0.0, 1.0, 1.0);
//
//    for (size_t object_id = 0; object_id < all_Map_objs.size(); ++object_id)
//    {
//        MapObjects *obj_landmark = all_Map_objs[object_id];
//    }
//}
