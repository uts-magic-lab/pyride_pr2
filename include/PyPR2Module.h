/*
 *  PyPR2Module.h
 *  PyRIDE
 *
 *  Created by Xun Wang on 18/06/12.
 *  Copyright 2012 Galaxy Network. All rights reserved.
 *
 */

#ifndef PY_PR2_MODULE_H
#define PY_PR2_MODULE_H

#include <PyModuleStub.h>

namespace pyride {

#ifdef WITH_RHYTH_DMP
struct DMPTrajCmdData {
  std::string name;
  float amplitude;
  float system_freq;
  int sampling_freq;
  int cycles;
  double pos_x;
  double pos_y;
  double pos_z;
  double ori_w;
  double ori_x;
  double ori_y;
  double ori_z;
};

typedef std::vector<DMPTrajCmdData> DMPTrajCmdList;
#endif

class PyPR2Module : public PyModuleExtension
{
public:
  static PyPR2Module * instance();
  virtual ~PyPR2Module();
  
  void invokeBaseScanCallback( PyObject * arg );
  void invokeTiltScanCallback( PyObject * arg );
  
  void setBaseScanCallback( PyObject * obj );
  void setTiltScanCallback( PyObject * obj );

#ifdef WITH_PR2HT
  void setObjectDTCallback( PyObject * detectcb, PyObject * trackcb );
  
  void invokeObjectDetectionCallback( PyObject * arg );
  void invokeObjectTrackingCallback( PyObject * arg );
#endif

#ifdef WITH_RHYTH_DMP
  void setTrajectoryInputCallback( PyObject * inputcb );
  void invokeTrajectoryInputCallback( PyObject * arg );
#endif

private:
  static PyPR2Module * s_pyPR2Module;
  
  PyObject * baseScanCB_;
  PyObject * tiltScanCB_;
  
#ifdef WITH_PR2HT
  PyObject * objectDetectCB_;
  PyObject * objectTrackCB_;
#endif

#ifdef WITH_RHYTH_DMP
  PyObject * trajInputCB_;
#endif

  PyPR2Module();
  PyObject * createPyModule();
};

} // namespace pyride

#endif // PY_PR2_MODULE_H
