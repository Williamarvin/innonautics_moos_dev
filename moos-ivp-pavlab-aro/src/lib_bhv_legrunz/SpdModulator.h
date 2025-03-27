/*****************************************************************/
/*    NAME: Michael Benjamin                                     */
/*    ORGN: Dept of Mechanical Engineering, MIT, Cambridge MA    */
/*    FILE: SpdModulator.h                                       */
/*    DATE: June 29th, 2021, Broken out to separate class        */
/*    DATE: June 8th, 2023, Became generic speed modulator       */
/*                                                               */
/* This is unreleased BETA code. No permission is granted or     */
/* implied to use, copy, modify, and distribute this software    */
/* except by the author(s), or those designated by the author.   */
/*****************************************************************/
 
#ifndef SPD_MODULATOR_HEADER
#define SPD_MODULATOR_HEADER

#include <string>
#include <vector>
#include <stdlib.h>  // atof()

class SpdModulator {
public:
  SpdModulator();
  ~SpdModulator() {}

public: // setters
  bool   setFullStopRng(double);
  bool   setSlowerRng(double);
  bool   setFasterRng(double);
  bool   setFullLagRng(double);
  void   setMaxCompression(double);
  void   setLagSpeedDelta(double);
  bool   setParam(std::string, std::string);

public: // getters
  double getFasterRng() const  {return(m_faster_rng);}
  double getFullLagRng() const {return(m_full_lag_rng);}
  double getSlowerRng() const  {return(m_slower_rng);}
  double getFullStopRng() const    {return(m_full_stop_rng);}
  double getLagSpeedDelta() const  {return(m_lag_speed_delta);}
  double getMaxCompression() const {return(m_max_compression);}

  std::string status();
  std::string getCorrectionMode() const {return(m_correction_mode);}
  
  double getSpdFromPolicy(double, double);

  bool   compress(double pct);

  bool   atIdealRng(double);

  std::string getSpec() const;
  std::string getTerse() const;
  
private: // Config params
  double m_full_stop_rng;
  double m_slower_rng;
  double m_faster_rng;
  double m_full_lag_rng;
  
  double m_lag_speed_delta;
  double m_max_compression;
  
private: // State vars

  std::string m_correction_mode;
};

SpdModulator string2SpdModulator(std::string);

#endif


