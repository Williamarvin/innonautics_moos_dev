/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GraphNode.h                                     */
/*    DATE: March 5th, 2022                                 */
/************************************************************/

#ifndef GRAPHNODE_HEADER
#define GRAPHNODE_HEADER

#include <vector>


class Node
{
 public:
  Node();
  ~Node();
  void resetNode();

  // Getters
  int getNumber() const {return(m_number);};
  int getParent() const {return(m_parent);};
  double getF() const {return(m_f);};
  double getG() const {return(m_g);};
  double getH() const {return(m_h);};
  
  std::vector<std::size_t> getNeighbors() const {return(m_neighbors);};
  std::vector<double> getPos() const {return(m_pos);};

  bool isGoal() const {return(m_isGoal);};
  bool isStart() const {return(m_isStart);};
  
  // Setters
  void setNumber(int n) {m_number = n; return;};
  void setParent(int p) {m_parent = p; return;};
  void setF(double f) {m_f = f; return;};
  void setG(double g) {m_g = g; return;};
  void setH(double h) {m_h = h; return;};
  
  void setNeighbors(std::vector<std::size_t> v) {m_neighbors = v; return;};
  void setPos(std::vector<double> v) {m_pos = v; return;};

  void setIsGoal(bool b) {m_isGoal = b; return;};
  void setIsStart(bool b) {m_isStart = b; return;};
 


 protected:
  int m_number;  // the number of this node
  int m_parent;  // node number of the parent node. 
  double m_f, m_g, m_h;  // Cost values. f = g + h, h = hueristic, g = cost.
  std::vector<std::size_t> m_neighbors; // all the neighbors of this node
  std::vector<double> m_pos;  // the location of the node.  
  bool m_isGoal, m_isStart;

  
};






#endif 
