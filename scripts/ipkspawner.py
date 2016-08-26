import PyREEM
import json
import sys
from threading import Thread

class dumbsig:
  SIGINT = 1
  SIG_IGN = 0

  @classmethod
  def signal(*args):
    pass

class IPKSpawner( object ):
  def __init__( self ):
    self.app = None
    self.is_initialised = None
    try:
      from ipykernel import kernelapp
      self.kernelapp = kernelapp
    except:
      return
    kernelapp.signal = dumbsig
    self.app = kernelapp.IPKernelApp.instance()
    self.app.no_stdout = self.app.no_stderr = False

  def spawnkernel( self, argstr ):
    if not self.app:
      return False

    if self.is_initialised:
      return True

    argv = [ 'ipykernel', '-f', argstr, '--debug' ]
    print( "init with {}".format( argv ) )
    self.app.initialize( argv )
    self.app.kernel.pre_handler_hook = dumbsig.signal
    self.app.kernel.post_handler_hook = dumbsig.signal
    #main = self.app.kernel.shell._orig_sys_modules_main_mod
    #if main is not None:
       #sys.modules[self.app.kernel.shell._orig_sys_modules_main_name] = main

    # load the calling scope if not given
    #from IPython.utils.frame import extract_module_locals
    #(caller_module, caller_locals) = extract_module_locals(1)

    #self.app.kernel.user_module = caller_module
    #self.app.kernel.user_ns = caller_locals
    #self.app.shell.set_completer_frame()
    self.is_initialised = Thread(target=self._start_kernel_thread)
    self.is_initialised.start()

  def _start_kernel_thread( self ):
    if not self.app:
      return

    PyREEM.sendMessageToNode( 'jupyter', 'start' )
    self.app.start()
    self.app.shell_socket.close()
    self.app.stdin_socket.close()
    self.app.control_socket.close()
    self.app.iopub_socket.close()
    self.app.heartbeat.socket.close()
    self.kernelapp.IPKernelApp.clear_instance()
    PyREEM.sendMessageToNode( 'jupyter', 'stop' )
    print( "reinitiate app instance" )
    self.app = self.kernelapp.IPKernelApp.instance()
    self.is_initialised = None

sys.path.append( '/usr/local/lib/python2.7/dist-packages' )
