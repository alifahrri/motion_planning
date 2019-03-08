#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sympy
from code_template import *
from generator import *

class CodeGenerator() :
    def __init__(self) : 
        ## initial placeholder
        self.symbols = {
            ## system matrix
            'A' : sympy.Matrix(), 
            ## input matrix
            'B' : sympy.Matrix(),
            ## input weight
            'R' : sympy.Matrix(),
            ## exponential of system matrix
            'eAt' : sympy.Matrix(),
            ## composite matrix
            'composite_matrix' : sympy.Matrix(),
            ## exponential of composite matrix
            'exp_composite_matrix' : sympy.Matrix(),
            ## d
            'd' : sympy.Matrix(),
            ## x_bar
            'x_bar' : sympy.Matrix(),
            ## controllability gramian
            'gramian' : sympy.Matrix(),
            ## cost 
            'cost' : sympy.Matrix(),
            ## derivative of the cost
            'd_cost' : sympy.Matrix(),
            ## dimensionality of the system 
            'dim' : sympy.Matrix(),
            ## dimensionality of the input
            'u_dim' : sympy.Matrix()
        }
        self.ccode = {
            'A' : None,
            'B' : None,
            'C' : None,
            'gramian' : None,
            'jordan' : None,
            'exp' : None,
            'cost' : None,
            'dcost' : None,
            'variable' : None,
            'set_variable' : None,
            'vr' : None,
            'aabb' : None,
            'daabb' : None,
            'cmp_j' : None,
            'cmp_p' : None,
            'exp_cmp' : None,
            'cmp.A' : None
        }
    def generate_controller(self, mA, mB) :
        _, self.symbols, self.ccode = generate_controller(mA, mB)
    def generate_ccode(self, model_name, dim, u_dim) :
        code = generate_ccode(model_name, self.symbols['dim'], u_dim, self.ccode)
        return code
    def generate_test_ccode(self, model_name, dim, u_dim) :
        code = generate_test_ccode(model_name, self.symbols['dim'], u_dim, self.ccode)
        return code
    def generate_latex(self, model_name) :
        pass