# run with python2 setup.py build_ext --inplace
from distutils.core import setup, Extension
setup(name='dcmimu',
      ext_modules=[
        Extension('dcmimu',
                  ['bb_dcmimu.c']
                  )
        ]
)