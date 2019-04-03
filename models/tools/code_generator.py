#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sympy
from code_template import *
from generator import *

import pylatex	

Package = pylatex.package.Package

class WrapEquation(pylatex.base_classes.CommandBase) :
    __latex_name = 'wrapequation'
    packages = [Package('amsmath'), Package('breqn')]

class CmdEquation(pylatex.base_classes.CommandBase) :
    __latex_name = 'cmdequation'
    packages = [Package('amsmath'), Package('amssymb'), Package('mathtools')]

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
        self.latex_doc = None
        self.latex_str = ''
    def generate_nonlinear_controller(self, mA, mB, trg, var) :
        generate_nonlinear_controller(mA, mB, trg, var)
    def generate_controller(self, mA, mB) :
        _, self.symbols, self.ccode = generate_controller(mA, mB)
    def generate_ccode(self, model_name, dim, u_dim) :
        code = generate_ccode(model_name, self.symbols['dim'], u_dim, self.ccode)
        return code
    def generate_test_ccode(self, model_name, dim, u_dim) :
        code = generate_test_ccode(model_name, self.symbols['dim'], u_dim, self.ccode)
        return code
    def generate_latex(self, model_name) :
        symbols = self.symbols
        italic = pylatex.utils.italic
        latex, NoEscape, Arguments = sympy.latex, pylatex.utils.NoEscape, pylatex.base_classes.Arguments
        doc = self.latex_doc
        doc = pylatex.Document()
        self.eq_cmd = {
            'wrapequation' : pylatex.UnsafeCommand(
                'newcommand',
                '\wrapequation',
                options=1,
                extra_arguments=r'\begin{dmath} #1 \end{dmath}'
            ),
                'cmdequation' : pylatex.UnsafeCommand(
                'newcommand',
                '\cmdequation',
                options=1,
                extra_arguments=r'\begin{equation} #1 \end{equation}'
            )
        }
        doc.append(self.eq_cmd['wrapequation'])
        doc.append(self.eq_cmd['cmdequation'])
        title = 'Derivation of Fixed Final-State Free Final-Time Control for {}'.format(model_name)
        author = 'Automatically Generated Text'
        doc.preamble.append(pylatex.Command('title', title))
        doc.preamble.append(pylatex.Command('author', author))
        doc.preamble.append(pylatex.Command('date', pylatex.utils.NoEscape(r'\today')))
        doc.append(pylatex.utils.NoEscape(r'\maketitle'))
        with doc.create(pylatex.Section('Fixed Final-State Free Final-Time Control for General Linear System')) :
            description = r'''
            The linear dynamics of the robot is expressed using :

            \begin{equation} \label{state_space_eq}
            \dot{\mathbf{x}}=A\mathbf{x}+B\mathbf{u}
            \end{equation}

            Where $\boldsymbol{x}$ is the state of robot, $\boldsymbol{u}$ is the input of the robot, and $A\in \mathbb{R}^{nxn}$ is the system matrix, and $B \in \mathbb{R}^{nxm}$ is the input matrix. The trajectory of the robot is defined by a tuple $\pi = (\boldsymbol{x}[], \boldsymbol{u}[], \tau)$. Where $\tau$ represents the arrival time of the given trajectory.
            The cost of a trajectory $\pi$ is defined as :

            \begin{equation} \label{cost_eq}
            c(\pi) = \int_{0}^{\tau}(1+\boldsymbol{u}(t)^{T}R\boldsymbol{u}(t))dt
            \end{equation}
            where $R\in\mathbb{R}^{mxm}$ is the input weighting matrix. The cost function above penalize both the duration of the trajectory, as represented by the term $1$ which will be integrated to $\tau$ which is the duration of the trajectory, and the exerted control input $\boldsymbol{u}(t)$.

            For a fixed final time, fixed final state optimal control problem where arrival time $\tau$, initial state $\boldsymbol{x_{i}}$ and final state $\boldsymbol{x_{f}}$ are given, the open-loop optimal control policy minimizing the cost function above is given by :
            \begin{equation} \label{input_eq}
            \boldsymbol{u}(t) = R^{-1}B^{T}e^{A^{T}(\tau-t)}G(\tau)^{-1}(\boldsymbol{x}_{f}-\boldsymbol{\bar{x}}(\tau))
            \end{equation}
            where $G(t)$ is the weighted controllability Gramian \cite{lewis2012optimal} defined as :

            \begin{equation} \label{gramian_eq}
            G(t)=\int_{0}^{t}e^{A(t-t^{'})}BR^{-1}B^{T}e^{A^{T}(t-t^{'})}dt^{'}
            \end{equation}
            and $\boldsymbol{\bar{x}}(t)$ described the state at time $t$ when no control input is applied :

            \begin{equation} \label{xbar_eq}
            \boldsymbol{\bar{x}}(t) = e^{At}\boldsymbol{x}_{i}.
            \end{equation}

            In \cite{webb2013kinodynamic}, the analysis above is extended to solve fixed final state, free final time optimal control problem. The closed form of cost function is found by filling the control policy above and evaluating the integral:

            \begin{equation} \label{extended_cost_eq}
            c(\tau) = \tau + (\boldsymbol{x}_f - \boldsymbol{\bar{x}}(\tau))^{T}G(\tau)^{-1}(\boldsymbol{x}_f-\boldsymbol{\bar{x}}(\tau))
            \end{equation}
            and the optimal arrival time  :

            \begin{equation} \label{tau_star_eq}
            \tau^{*} = argmin\left \{ \tau > 0 \right \}c(\tau)
            \end{equation}
            and the optimal arrival time $\tau^{*}$ by derive $c(\tau)$ with respect to $\tau$ and solving for $\dot{c}(\tau) = 0$. Where the derivative of $c(\tau)$ is given by :

            \begin{equation} \label{derivative_cost_eq}
            \dot{c}(\tau) = 1 - 2(A\boldsymbol{x}_f)^{T}\boldsymbol{d}(\tau)-\boldsymbol{d}(\tau)^{T}BR^{-1}B^{T}\boldsymbol{d}(\tau)
            \end{equation}
            and $\boldsymbol{d}(\tau)$ is defined as :

            \begin{equation} \label{d_sym_eq}
            \boldsymbol{d}(\tau) = G(\tau)^{-1}(\boldsymbol{x}_f-\boldsymbol{\bar{x}}(\tau))
            \end{equation}

            After the optimal arrival time is found, the optimal trajectory connecting $\boldsymbol{x}_i$ to $\boldsymbol{x}_f$ could be solved as follow. 
            define :
            
            \begin{equation} \label{y_sym_eq}
            \boldsymbol{y}(t) = e^{A^{T}(\tau^{*})}\boldsymbol{d}(\tau^{*})
            \end{equation}
            which is the solution of :

            \begin{equation} \label{ydot_eq}
            \\\dot{\boldsymbol{y}}(t) = -A^{T}\boldsymbol{y}(t),
            \\\boldsymbol{y}(\tau^{*}) = \boldsymbol{d}(\tau^{*})
            \end{equation}
            such that the optimal control policy becomes :

            \begin{equation} \label{ut_eq}
            \boldsymbol{u}(t) = R^{-1}B^{T}\boldsymbol{y}(t)
            \end{equation}
            and filling the optimal control policy above to eq. \ref{state_space_eq} gives the differential equation for $\boldsymbol{x}$:

            \begin{equation} \label{extended_xdot_eq}
            \\\dot{\boldsymbol{x}}(t) = A\boldsymbol{x}+BR^{-1}B^{T}\boldsymbol{y}(t), \\\boldsymbol{x}(\tau^{*})=\boldsymbol{x_f}
            \end{equation}
            combining the two differential equation of eq. \ref{extended_xdot_eq} and eq. \ref{ydot_eq} gives :

            \begin{equation} \label{composite_derivative_eq}
            \begin{bmatrix}
            \dot{\boldsymbol{x}}(t)\\
            \dot{\boldsymbol{y}}(t)
            \end{bmatrix}
            =
            \begin{bmatrix}
            A & BR^{-1}B^{T}\\
            0 & -A^{T}
            \end{bmatrix}
            \begin{bmatrix}
            \boldsymbol{x}(t) \\
            \boldsymbol{y}(t)
            \end{bmatrix}
            \end{equation}
            and the solution to the composite differential equation above is given by:

            \begin{equation} \label{composite_state_space_eq}
            \begin{bmatrix}
            \boldsymbol{x}(t)\\
            \boldsymbol{y}(t)
            \end{bmatrix}
            =
            e^{M(t-\tau^{*})}
            \begin{bmatrix}
            \boldsymbol{x}_f \\
            \boldsymbol{d}(\tau^{*})
            \end{bmatrix}
            \end{equation}
            where

            \begin{equation} \label{m_sym_eq}
            M = \begin{bmatrix}
            A & BR^{-1}B^{T}\\
            0 & -A^{T}
            \end{bmatrix}
            \end{equation}

            The equation \ref{composite_state_space_eq} above gives the optimal trajectory $\boldsymbol{x}$ for general system. In the following section, we derived a fixed-state free-final-time controller for double integrator model.
            '''
            doc.append(NoEscape(description))
        with doc.create(pylatex.Section('Fixed Final-State Free Final-Time Control for {}'.format(model_name))) :
            doc.append('The linear dynamics of the system : ')
            sys = 'A = {};\; B = {}'.format(latex(symbols['A']), latex(symbols['B']))
            doc.append(CmdEquation(arguments=Arguments(NoEscape(sys))))
            doc.append('Define the input weight matrix, $R$, as : ')
            R = 'R = {}'.format(latex(symbols['R']))
            doc.append(CmdEquation(arguments=Arguments(NoEscape(R))))
            doc.append('Exponential of the system matrix $A$ is given by : ')
            eAt = 'e^{{At}} = {}'.format(latex(symbols['eAt']))
            doc.append(WrapEquation(arguments=Arguments(NoEscape(eAt))))
            doc.append('The cost function of the system : ')
            cost = r'c(\tau) = {}'.format(latex(symbols['cost'][0]))
            doc.append(WrapEquation(arguments=Arguments(NoEscape(cost))))
            doc.append('The first derivative of the cost function : ')
            dcost = r'\dot{{c}}(\tau) = {}'.format(latex(symbols['d_cost'][0]))
            doc.append(WrapEquation(arguments=Arguments(NoEscape(dcost))))
            doc.append(NoEscape(r'define $\boldsymbol{d}(\tau)$ as : '))
            d = r'\boldsymbol{{d}}(\tau) = {}' .format(latex(symbols['d']))
            doc.append(WrapEquation(arguments=Arguments(NoEscape(d))))
            doc.append('The '), doc.append(italic('weighted controllability Gramian ')), doc.append(' is expressed as : ')
            G = 'G = {}'.format(latex(symbols['gramian']))
            doc.append(WrapEquation(arguments=Arguments(NoEscape(G))))
            doc.append('The composite system expressing the open loop trajectory is defined by : ')
            cmp_sys_label = [
                r'\begin{bmatrix} \boldsymbol{x}(t)\\ \boldsymbol{y}(t) \end{bmatrix}',
                r'\begin{bmatrix} \boldsymbol{x}_f \\ \boldsymbol{d}(\tau^{*}) \end{bmatrix}'
            ]
            cmp_sys = '{} = {} {}'.format(cmp_sys_label[0], latex(symbols['exp_composite_matrix']), cmp_sys_label[1])
            doc.append(WrapEquation(arguments=Arguments(NoEscape(cmp_sys))))
        self.latex_doc = doc
        return doc.dumps()
    def generate_pdf(self, filename) :
        self.latex_doc.generate_pdf(filename, clean_tex=False)
