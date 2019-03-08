#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sympy
from code_template import *
		
def generate_controller(mA, mB) :
  r, t, b, tau = sympy.symbols('r t b tau')
  #r, t, po, vo, pf, vf, b, tau = sympy.symbols('r t po vo pf vf b tau')

  n = len(mA)
  A, B = sympy.Matrix(mA), sympy.Matrix(mB)
  rlist = []
  for i in range(len(mB[0])) :
    rv = []
    for j in range(len(mB[0])) :
      rv.append(0)
    rv[i] = r
    rlist.append(rv)
  # R = sympy.Identity()
  R = sympy.Matrix(rlist)
  cmp_mat = (A.row_join(B*R.inv()*B.transpose())).col_join(sympy.Matrix.zeros(A.rows,A.cols).row_join(-A.transpose()))
  exp_cmp_mat = sympy.exp(cmp_mat*t)
  print R, '\n', cmp_mat
  [cmp_P, cmp_J] = cmp_mat.jordan_form()

  x = []
  for i in range(n) :
    x.append(sympy.symbols('x%s'%i))
  xi_list, xf_list = [], []

  if_var_str, if_set_str = 'Scalar ', ''
  for i in range(n) :
    if_var_str = if_var_str + 'x%si'%i + ', ' + 'x%sf'%i
    if_set_str = if_set_str + 'x%si = xi(%s); x%sf = xf(%s); ' %(i,i,i,i)
    if i < n-1 :
      if_var_str = if_var_str + ', '
    xi_list.append([sympy.symbols('x%si'%i)])
    xf_list.append([sympy.symbols('x%sf'%i)])
  if_var_str = if_var_str + '; '

  xi = sympy.Matrix(xi_list)
  xf = sympy.Matrix(xf_list)
  xbar = sympy.exp(A*t)*xi

  G = sympy.integrate(sympy.exp(A*(t-tau))*B*R.inv()*B.transpose()*sympy.exp(A.transpose()*(t-tau)), (tau, 0, t))
  d = G.inv() * (xf - xbar)

  t1 = (2*A*xf).transpose()*d
  t2 = d.transpose()*B*R.inv()*B.transpose()*d
  c = sympy.Matrix([t]) + (xf -xbar).transpose() * G.inv() * (xf - xbar)
  dc = sympy.Matrix([1])- t1 - t2

  aabb, d_aabb = [], []
  for i in range(n) :
    aabb.append([xbar[i]-sympy.sqrt(G[i,i]*(b-t)), xbar[i]+sympy.sqrt(G[i,i]*(b-t))])
    d_aabb.append([sympy.diff(aabb[i][0],t), sympy.diff(aabb[i][1], t)])

  vr = sympy.Matrix([sympy.pi**2*(G*(b-t)).det()])
  eat = sympy.exp(A*t)
  [P, J] = A.jordan_form()

  C = sympy.Identity(n)

  cmp_mat_str = 'A << '
  exp_cmp_mat_str = 'eAt << '
  cmp_P_str = 'P << '
  cmp_J_str = 'D << '
  for i in range(cmp_J.rows) :
    for j in range(cmp_J.cols) :
      exp_cmp_mat_str += sympy.ccode(exp_cmp_mat[i,j])
      cmp_mat_str += sympy.ccode(cmp_mat[i,j])
      cmp_J_str += sympy.ccode(cmp_J[i,j])
      cmp_P_str += sympy.ccode(cmp_P[i,j])
      if (i+1)*(j+1)-1 < cmp_J.cols*cmp_J.rows-1 :
        cmp_J_str += ', '
        cmp_P_str += ', '
        cmp_mat_str += ', '
        exp_cmp_mat_str += ', '
  cmp_P_str += ';'
  cmp_J_str += ';'
  exp_cmp_mat_str += ';'
  cmp_mat_str += ';'

  gram_str = 'G << '
  eat_str = 'eAt << '
  p_str, j_str = 'P << ', 'J << '
  a_str, b_str, c_str = 'A << ', 'B << ', 'C << '
  for i in range(n) :
    for j in range(n) :
      a_str = a_str + sympy.ccode(A[i,j])
      c_str = c_str + sympy.ccode(C[i,j])
      gram_str = gram_str + sympy.ccode(G[i,j])
      p_str = p_str + sympy.ccode(P[i,j])
      j_str = j_str + sympy.ccode(J[i,j])
      eat_str = eat_str + sympy.ccode(eat[i,j])
      if ((i+1)*(j+1) < n*n-1):
        gram_str = gram_str + ', '
        p_str = p_str + ', '
        j_str = j_str + ', '
        a_str = a_str + ', '
        c_str = c_str + ', '
        eat_str = eat_str + ', '
  for i in range(B.rows) :
    for j in range(B.cols) :
      b_str = b_str + sympy.ccode(B[i,j])
      if (i+1)*(j+1)-1 < B.rows*B.cols-1 :
        b_str = b_str + ', '
  gram_str = gram_str + ';'
  a_str = a_str + ';'
  b_str = b_str + ';'
  c_str = c_str + ';'
  p_str = p_str + ';'
  j_str = j_str + ';'
  eat_str = eat_str + ';'
  jordan_str = p_str + '\n' + j_str
  cost_str = 'cost = %s;' % sympy.ccode(c[0])
  dc_str = 'd_cost = %s;' % sympy.ccode(dc[0])
  vr_str = 'vr = %s;' % sympy.ccode(vr[0])
  aabb_str = 'aabb << '
  d_aabb_str = 'd_aabb << '
  for i in range(n) :
    for j in range(2) :
      aabb_str = aabb_str + sympy.ccode(aabb[i][j])
      d_aabb_str = d_aabb_str + sympy.ccode(d_aabb[i][j])
      if ((i+1)*(j+1) < n*2-1) :
        aabb_str = aabb_str + ', '
        d_aabb_str = d_aabb_str + ', '
  aabb_str = aabb_str + ';'
  d_aabb_str = d_aabb_str + ';'
  c_code = {
		# system matrix :
    'A' : a_str,
		# input matrix :
    'B' : b_str,
    'C' : c_str,
		# controllability gramian :
    'gramian' : gram_str,
    'jordan' : jordan_str,
    'exp' : eat_str,
    'cost' : cost_str,
    'dcost' : dc_str,
    'variable' : if_var_str,
    'set_variable' : if_set_str,
    'vr' : vr_str,
    'aabb' : aabb_str,
    'daabb' : d_aabb_str,
    'cmp_j' : cmp_J_str,
    'cmp_p' : cmp_P_str,
    'exp_cmp' : exp_cmp_mat_str,
    'cmp.A' : cmp_mat_str
  }
  symbols = {
    'A' : A,
    'B' : B,
    'R' : R,
    'eAt' : eat,
    'composite_matrix' : cmp_mat,
    'exp_composite_matrix' : exp_cmp_mat,
    'd' : d,
    'x_bar' : xbar,
    'gramian' : G,
    'cost' : c,
    'd_cost' : dc,
    'dim' : n,
    # 'u_dim' : u_dim
  }
  # return c_code
  # return [gram_str,jordan_str,eat_str,cost_str,dc_str,aabb_str,d_aabb_str,vr_str, if_var_str, if_set_str, a_str, b_str, c_str, cmp_J_str, cmp_P_str, exp_cmp_mat_str]
  return parse_ccode(c_code), symbols, c_code

def parse_ccode(ccode) :
  return [
    ccode['gramian'],
    ccode['jordan'],
    ccode['exp'],
    ccode['cost'],
    ccode['dcost'],
    ccode['aabb'],
    ccode['daabb'],
    ccode['vr'], 
    ccode['variable'],
    ccode['set_variable'], 
    ccode['A'], 
    ccode['B'], 
    ccode['C'], 
    ccode['cmp_j'], 
    ccode['cmp_p'], 
    ccode['exp_cmp']
  ]

def generate_ccode(model_name, dim, u_dim, code_dict) :
  d = code_dict
  return generate_cpp(model_name, dim, u_dim, d['gramian'], d['jordan'], d['exp'], d['cost'], d['dcost'], d['aabb'], d['daabb'], d['vr'], d['variable'], d['set_variable'], d['A'], d['B'], d['C'], d['cmp_j'], d['cmp_p'], d['exp_cmp'])

def generate_test_ccode(model_name, dim, u_dim, code_dict) :
  d = code_dict
  return generate_test_cpp(model_name, dim, u_dim, d['gramian'], d['jordan'], d['exp'], d['cost'], d['dcost'], d['aabb'], d['daabb'], d['vr'], d['variable'], d['set_variable'], d['A'], d['B'], d['C'], d['cmp_j'], d['cmp_p'], d['exp_cmp'])
