#ifndef _DATA_STRUCT_H
#define _DATA_STRUCT_H
#include <Eigen/Eigen>
#include <iostream>
#include <ros/console.h>
#include <ros/ros.h>
#include <vector>

#define inf 1 >> 20
class
    GridNode;  // 必须要在定义别名前先声明，要不编译器不知道GrideNode是什么！！！！
class Cube;
using GridNodePtr = GridNode*;

/*
        P1----------------P2                           ^y
           |                           |                              |
           |                           |                              |
           |                           |                              |
        P4----------------P3                            ------------------> x
*/
class Cube {
 private:
 protected:
 public:
  Eigen::MatrixXd vertex;
  Eigen::Vector2d center;
  bool valid;
  double t;

  std::vector<std::pair<double, double>> box;  // box records the  cube length

  Cube() {
    center = Eigen::Vector2d::Zero(2);
    vertex = Eigen::MatrixXd::Zero(4, 2);

    valid = true;
    t = 0.0;
  }
  ~Cube(){};

  Cube(Eigen::MatrixXd _vertex, Eigen::Vector2d _center) {
    vertex = _vertex;
    center = _center;
    valid = true;
    t = 0.0;
    box.resize(2);
  }

  void setVertex(Eigen::MatrixXd _vertex, double resolution) {
    vertex = _vertex;
    // x-axis
    vertex(0, 0) -= resolution / 2.0;  // P1
    vertex(1, 0) += resolution / 2.0;  // P2
    vertex(2, 0) += resolution / 2.0;  // P3
    vertex(3, 0) -= resolution / 2.0;  // P4

    // y-axis
    vertex(0, 1) += resolution / 2.0;  // P1
    vertex(1, 1) += resolution / 2.0;  // P2
    vertex(2, 1) -= resolution / 2.0;  // P3
    vertex(3, 1) -= resolution / 2.0;  // P4

    // center(0) = (vertex(0,0) + vertex(1,0)) / 2;
    // center(1) = (vertex(0,1) + vertex(3,1)) / 2;

    setBox();
  }

  void setBox() {
    box.clear();
    box.resize(2);
    box[0] = std::make_pair(
        vertex(0, 0), vertex(1, 0));  // x length of cube   P1-----P2
    box[1] = std::make_pair(
        vertex(3, 1), vertex(0, 1));  // ylength of cube   P1-----P4
  }

  void printBox() {
    std::cout << "the center of Cube is " << center << std::endl;
    std::cout << "the x length of Cube is: \n " << (vertex(1, 0) - vertex(0, 0))
              << std::endl;
    std::cout << "the y length of Cube is: \n " << (vertex(0, 1) - vertex(3, 1))
              << std::endl;
  }
};

class GridNode {
 private:
 protected:
 public:
  int id;  // 1-- in the openset     0--not detected before         -1-- in the
           // close set
  Eigen::Vector2i index;
  Eigen::Vector2d coor;
  double gscore;
  double fscore;
  GridNodePtr ParentNode;
  std::multimap<double, GridNodePtr>::iterator NodeItself;
  int isOccupy;  // 1 -- ObstacelNode 0--FreeNode
  bool inflatOccpy;
  GridNode(Eigen::Vector2i _index, Eigen::Vector2d _coor) {
    id = 0;
    index = _index;
    coor = _coor;
    gscore = inf;
    fscore = inf;
    ParentNode = nullptr;
    isOccupy = 0;
    inflatOccpy = false;
  }
  ~GridNode(){};
  GridNode() = default;
};

/*
struct GridNode
{

    int id; // 1-- in the openset     0--not detected before         -1-- in the
close set Eigen::Vector2d index; Eigen::Vector2d coor; double gscore; double
fscore; GridNodePtr  ParentNode; std::multimap<double , GridNodePtr>::iterator
NodeItself; int isOccupy; // 1 -- ObstacelNode 0--FreeNode
    GridNode(Eigen::Vector2d _index , Eigen::Vector2d _coor)
{
    id = 0;
    index = _index;
    coor = _coor;
    gscore = inf;
    fscore = inf;
    ParentNode = nullptr;
    isOccupy = 0;
}
    ~GridNode(){};
    GridNode(){};

};

*/
#endif