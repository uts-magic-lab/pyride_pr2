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

class PyPR2Module : public PyModuleExtension
{
public:
  static PyPR2Module * instance();
  virtual ~PyPR2Module();
  
  void invokeBaseScanCallback( PyObject * arg );
  void invokeTiltScanCallback( PyObject * arg );
  
  void setBaseScanCallback( PyObject * obj );
  void setTiltScanCallback( PyObject * obj );

  void setObjectDTCallback( PyObject * detectcb, PyObject * trackcb );
  
  void invokeObjectDetectionCallback( PyObject * arg );
  void invokeObjectTrackingCallback( PyObject * arg );

private:
  static PyPR2Module * s_pyPR2Module;
  
  PyObject * baseScanCB_;
  PyObject * tiltScanCB_;
  
  PyObject * objectDetectCB_;
  PyObject * objectTrackCB_;

  PyPR2Module();
  PyObject * createPyModule();
};

} // namespace pyride

#endif // PY_PR2_MODULE_H
