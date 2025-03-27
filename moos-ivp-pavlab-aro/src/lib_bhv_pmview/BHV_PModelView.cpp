/*********************************************************/
/*    NAME: Michael Benjamin                             */
/*    ORGN: Dept of Mechanical Eng, MIT Cambridge MA     */
/*    FILE: BHV_PModelView.cpp                           */
/*    DATE: Nov 3rd 2023                                 */
/*********************************************************/

#include <iostream>
#include "BHV_PModelView.h"
#include "MBUtils.h"
#include "BuildUtils.h"

using namespace std;

//-----------------------------------------------------------
// Procedure: Constructor

BHV_PModelView::BHV_PModelView(IvPDomain gdomain) : 
  IvPBehavior(gdomain)
{
  this->setParam("descriptor", "pview");

  m_domain = subDomain(m_domain, "course,speed");

  m_edge_color = "white";
  m_vertex_color = "white";
  m_vertex_size = 3;
}

//-----------------------------------------------------------
// Procedure: setParam()

bool BHV_PModelView::setParam(string param, string val) 
{
  if(IvPBehavior::setParam(param, val))
    return(true);

  bool handled = false;
  if(param == "edge_color")
    handled = setColorOnString(m_edge_color, val);
  else if(param == "vertex_color")
    handled = setColorOnString(m_vertex_color, val);
  else if(param == "vertex_size")
    handled = setNonNegDoubleOnString(m_vertex_size, val);
  
  return(handled);
}

//-----------------------------------------------------------
// Procedure: onSetParamComplete()

void BHV_PModelView::onSetParamComplete()
{
}


//-----------------------------------------------------------
// Procedure: onRunState()

IvPFunction *BHV_PModelView::onRunState() 
{
  updateInfoIn();
  drawTurnPath();
  return(0);
}

//-----------------------------------------------------------
// Procedure: onRunToIdleState

void BHV_PModelView::onRunToIdleState() 
{
  eraseTurnPath();
}

//-----------------------------------------------------------
// Procedure: updateInfoIn()
//   Purpose: Update relevant to the behavior from the info_buffer.
//   Returns: true if no relevant info is missing from the info_buffer.
//            false otherwise.

bool BHV_PModelView::updateInfoIn()
{
  // =================================================
  // Part 1: Update ownship current heading and speed
  // =================================================
  if(!getBufferDoubleValX("NAV_X", m_osx))
    return(false);
  if(!getBufferDoubleValX("NAV_Y", m_osy))
    return(false);
  if(!getBufferDoubleValX("NAV_SPEED", m_osv))
    return(false);
  if(!getBufferDoubleValX("NAV_HEADING", m_osh))
    return(false);
  
  return(true);
}


//-----------------------------------------------------------
// Procedure: buildOF()

IvPFunction *BHV_PModelView::buildOF() 
{
  return(0);
}

//---------------------------------------------------------------
// Procedure: drawTurnPath()

void BHV_PModelView::drawTurnPath()
{
  
  
}

//---------------------------------------------------------------
// Procedure: eraseTurnPath()

void BHV_PModelView::eraseTurnPath()
{
}
