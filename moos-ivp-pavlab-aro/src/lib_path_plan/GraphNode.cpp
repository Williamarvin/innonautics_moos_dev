/************************************************************/
/*    NAME: Tyler Paine                                     */
/*    ORGN: MIT, Cambridge MA                               */
/*    FILE: GraphNode.cpp                                   */
/*    DATE: March 5th, 2022                                 */
/************************************************************/


#include "GraphNode.h"

//---------------------------------------------------------
// Constructor

Node::Node()
{
  m_number = -1;
  m_parent = -1;
  m_f = -1;
  m_g = -1;
  m_h = -1;
  m_neighbors = std::vector<std::size_t>();
  m_pos = std::vector<double>();
  m_isGoal = false;
  m_isStart = false;
}


//---------------------------------------------------------
// Destructor

Node::~Node()
{
  
}

//------------------------------------------------------------
// Reset
void Node::resetNode()
{
  m_number = -1;
  m_parent = -1;
  m_f = -1;
  m_g = -1;
  m_h = -1;
  m_neighbors = std::vector<std::size_t>();
  m_pos = std::vector<double>();
  m_isGoal = false;
  m_isStart = false;
  
}

