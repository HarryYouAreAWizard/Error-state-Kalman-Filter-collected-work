Two ways of running the ESKF using simulations are implemented in this repo. 

"ESKF_and_rocketpy.ipynb" is a notebook running the ESKF with data generated with the notebook under "rocketpysims".

__main__.py generates data using a custom simulator and runs the ESKF with the generated data.

The ESKF is placed in its own repository. It must be compiled into a shared libray "libmain.so", from which the estimator_wrap.py can access it. in there, the path to "libmain.so" must be specified.


This repo is currently not very user friendly. This might be fixed someday
