#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sympy
from multipledispatch import dispatch
from code_template import *
import logging as log

# log.basicConfig()

@dispatch(str)
def generate_symbol(sym) :
  sym_list = sympy.symbols(sym)
  return sym_list

@dispatch(int, str)
def generate_symbol(n, fmt) :
  sym_list = [sympy.symbols(fmt.format(i)) for i in range(n)]
  return sym_list

def generate_init_final_state(n) :
  if_var_str, if_set_str = 'Scalar ', ''
  for i in range(n) :
    if_var_str = if_var_str + 'x%si'%i + ', ' + 'x%sf'%i
    if_set_str = if_set_str + 'x%si = xi(%s); x%sf = xf(%s); ' %(i,i,i,i)
    if i < n-1 :
      if_var_str = if_var_str + ', '
  if_var_str = if_var_str + '; '
  xi_list = generate_symbol(n, 'x{}i')
  xf_list = generate_symbol(n, 'x{}f')
  return xi_list, xf_list, if_var_str, if_set_str

def generate_state_matrices(xi_list, xf_list, A) :
  t = sympy.symbols('t')
  xi = sympy.Matrix(xi_list)
  xf = sympy.Matrix(xf_list)
  xbar = sympy.exp(A*t)*xi
  return xi, xf, xbar

@dispatch(int, sympy.Matrix, sympy.Matrix) 
def compute_state_equation(n, A, c) :
  t = generate_symbol('t tau')
  xi = sympy.Matrix(generate_symbol(n, 'x{}i'))
  xf = sympy.Matrix(generate_symbol(n, 'x{}f'))
  term = {
    1 : sympy.exp(A*t[0])*xi,
    2 : sympy.integrate(sympy.exp(A*(t[0]-t[1]))*c, (t[1], 0, t[0]))
  }
  xbar = term[1] + term[2]
  print term
  return xbar, xi, xf
  
@dispatch(int, sympy.Matrix)
def compute_state_equation(n, A) :
  t = generate_symbol('t')
  xi = sympy.Matrix(generate_symbol(n, 'x{}i'))
  xf = sympy.Matrix(generate_symbol(n, 'x{}f'))
  xbar = sympy.exp(A*t)*xi
  return xbar, xi, xf

def generate_state_symbols(n) :
  sym = ', '.join(['x{}'.format(i) for i in range(n)])
  print sym
  state_symbols = sympy.symbols(sym)
  state = sympy.sympify('Matrix([{}])'.format(sym))
  return sym, state_symbols, state

def generate_nonlinear_controller(mA, mB, trg, var, simplify=True) :
  n, m, A, B, R = generate_matrix(mA, mB)
  print 'n : {}; m : {}'.format(n,m)
  r, t, b, tau = sympy.symbols('r t b tau')
  def generate_input_vector(m) :
    u_list = ', '.join(['u{}'.format(u) for u in range(m)])
    u_sym = sympy.symbols(u_list)
    u_mat = sympy.sympify('Matrix([{}])'.format(u_list))
    return u_list, u_sym, u_mat
  u_list, u_sym, u_mat = generate_input_vector(m)
  sym, state_symbols, state = generate_state_symbols(n)
  xhat_sym = generate_symbol(n,'x{}_hat')
  xhat_dict = {}
  for i in range(n) :
    xhat_dict['x{}'.format(i)] = xhat_sym[i]
  for i in range(n) :
    for j in range(n) :
      v, t = var[i][j], trg[i][j]
      if len(v) and (('cos' in t) or ('sin' in t)) :
        print t, v
        A[i,j] *= sympy.sympify('{}({})'.format(t,xhat_dict[v]))
      elif len(v) :
        print t, v
        A[i,j] *= sympy.sympify('{}'.format(xhat_dict[v]))
  # f = A*state
  # linearization state
  xhat = sympy.Matrix(xhat_sym)
  print A
  print xhat
  matrices = {
    # nonlinear function 
    'f' : A*state,
    # linearized dynamic matrix A
    'A' : (A*xhat).jacobian(xhat),
    'B' : B,
    # linearization constant vector
    'c' : (A*xhat)-(A*xhat).jacobian(xhat)*xhat,
    # cost matrix
    'R' : R,
  }
  f, A, B, c = matrices['f'], matrices['A'], matrices['B'], matrices['c']
  state_fn = [f[i] for i in range(len(f))]
  xbar, xi, xf = compute_state_equation(n, A, c)
  xbar = sympy.simplify(xbar) if simplify else xbar

  print 'A : {}'.format(A)
  print 'B : {}'.format(B)
  print 'R : {}'.format(R)
  print 'u : {}'.format(u_mat)
  print 'f : {} + {}'.format([sympy.ccode(s) for s in state_fn], list(B*u_mat))
  print 'linearized matrix : {}'.format(A)
  print 'c (linearization constant) : {}'.format(c)
  print 'xi : {}'.format(xi)
  print 'xf : {}'.format(xf)
  print 'xbar : {}'.format(xbar)

  xi_list, xf_list, if_var_str, if_set_str = generate_init_final_state(n)

  t1, t2, cost, dcost, d, G = generate_cost_function(A, xf, B, R, xbar, False) 

  def simplify_mat(m) :
    rmat = sympy.Matrix(m[:,:])
    for i in range(m.rows) :
      for j in range(m.cols) :
        rmat[i,j] = sympy.simplify(m[i,j])
    return rmat

  ## takes too long time, skip ##
  # if simplify :
  #   print 'simplifying cost fn'
  #   cost = simplify_mat(cost)
  ## somehow this expr blocks forever, just don't ##
  # cost = sympy.Matrix(cost.rows, cost.cols, [sympy.simplify(cost[i]) for i in range(cost.rows*cost.cols)]) if simplify else cost

  print 'cost fn : {}'.format(cost)

  ## takes too long time, skip ##
  # if simplify :
    # print 'simplifying dcost fn'
    # dcost = simplify_mat(dcost)
  ## somehow this expr blocks forever, just don't ##
  # dcost = sympy.Matrix(dcost.rows, dcost.cols, [sympy.simplify(dcost[i]) for i in range(dcost.rows*dcost.cols)]) if simplify else dcost

  print 'cost fn derivative : {}'.format(dcost)

  if simplify :
    print 'simplifying gramian fn'
    G = simplify_mat(G)
  # G = sympy.Matrix(G.rows, G.cols, [sympy.simplify(G[i]) for i in range(G.rows*G.cols)]) if simplify else G
  print 'gramian : {}'.format(G)

  def generate_solution(A, B, R, c) :
    log.info('generate solution...')
    log.info('generating symbols...')
    t, tau_star, t_prime = generate_symbol('t tau_star t_prime')

    ## these computations takes forever, consider using jordan form
    # log.info('computing composite matrix...')
    # cmp_mat = sympy.simplify((A.row_join(B*R.inv()*B.transpose())).col_join(sympy.Matrix.zeros(A.rows,A.cols).row_join(-A.transpose())))
    # log.info('composite matrix : {}'.format(cmp_mat))
    # log.info('computing the exponetial of the composite matrix...')
    # exp_cmp_mat = sympy.simplify(sympy.exp(cmp_mat*t))
    # log.info('exponential of the composite matrix : {}'.format(exp_cmp_mat))

    print 'computing composite matrix...'
    cmp_mat = ((A.row_join(B*R.inv()*B.transpose())).col_join(sympy.Matrix.zeros(A.rows,A.cols).row_join(-A.transpose())))
    print 'computing composite matrix... OK'

    print 'computing jordan form...'
    cmp_P, cmp_J = cmp_mat.jordan_form()
    exp_J = sympy.exp(cmp_J*t)
    ## this expr takes a way long time
    # exp_cmp_mat = sympy.simplify(cmp_P * exp_J * cmp_P.inv())
    print 'computing jordan form... OK'

    ## dismiss, cause : can't get symbolic expr for exp_cmp_mat
    # print 'computing integration term...'
    # int_term = (exp_cmp_mat*sympy.Matrix(2*A.rows,1,list(c)+[0 for _ in range(A.rows)]))
    # print 'computing integration term... OK'

    log.info('wrapping up solutions...')
    solution = {
      ## our composit matrix to compute the solution
      'composite mat' : cmp_mat,
      ## break the matrix to jordan form for eazy computation on exponential term
      'jordan' : {
        'P' : cmp_P,
        'J' : cmp_J,
        'expJ' : exp_J
      },
      ## to integrate (numerically)
      # 'int' : int_term,
      'int' : {
        'vec' : sympy.Matrix(2*A.rows,1,list(c)+[0 for _ in range(A.rows)])
      }
    }
    log.info('generate solution... done')
    return solution
  
  print 'computing solution...'
  solution = generate_solution(A,B,R,c)
  print 'computing solution... OK'
  # for key, value in solution.items() :
    # print '{} : {}'.format(key, value)
  # print solution

  # print 'composite_matrix : {}'.format(cmp_mat)
  # print 'exponential of the composite matrix : {}'.format(exp_cmp_mat)

  def compute_jordan_form(A) :
    t = generate_symbol('t')
    P, J = A.jordan_form()
    expJ = sympy.exp(J*t)
    return P, J, expJ

  print 'computing jordan form of A'
  P, J, expJ = compute_jordan_form(A)

  symbols = {
    'A' : A, 'B' : B, 'R' : R,
    # 'eAt' : eat,
    # 'exp_composite_matrix' : exp_cmp_mat,
    'd' : d,
    'x_bar' : xbar,
    'gramian' : G,
    'cost' : cost,
    'd_cost' : dcost,
    'n' : n,
    'm' : m,
    'f' : f,
    'c' : c,
    'P' : P,
    'J' : J,
    'expJ' : expJ,
    'solution' : solution
    # 'u_dim' : u_dim
  }

  @dispatch(sympy.Matrix, str)
  def mat_ccode(mat, pfx, operator='<<') :
    ccode = sympy.ccode
    code_str = '{} {} '.format(pfx, operator)
    length = mat.rows*mat.cols
    for i in range(length) :
      code_str += ccode(mat[i])
      code_str += ';' if i == (length-1) else ', '
    return code_str
  @dispatch(sympy.MutableDenseMatrix, str)
  def mat_ccode(mat, pfx, operator='<<') :
    ccode = sympy.ccode
    code_str = '{} {} '.format(pfx, operator)
    length = mat.rows*mat.cols
    for i in range(length) :
      code_str += ccode(mat[i])
      code_str += ';' if i == (length-1) else ', '
    return code_str
  @dispatch(sympy.ImmutableDenseMatrix, str)
  def mat_ccode(mat, pfx, operator='<<') :
    ccode = sympy.ccode
    code_str = '{} {} '.format(pfx, operator)
    length = mat.rows*mat.cols
    for i in range(length) :
      code_str += ccode(mat[i])
      code_str += ';' if i == (length-1) else ', '
    return code_str

  @dispatch(str, str) 
  def mat_ccode(mat, pfx, operator='=') :
    return '{} {} {};'.format(pfx, operator, mat)

  @dispatch(int, str) 
  def mat_ccode(mat, pfx, operator='=') :
    return '{} {} {};'.format(pfx, operator, mat)

  ccode = {}
  for key, value in symbols.items() :
    # if not any([k in key for k in ['solution','cost']]) :
      # ccode[key] = sympy.ccode(value)
    if key == 'solution' :
      ccode[key] = {
        'composite mat' : mat_ccode(value['composite mat'], 'cmp_mat'),
        'jordan' : {
          'P' : mat_ccode(value['jordan']['P'],'P'),
          'J' : mat_ccode(value['jordan']['J'],'J'),
          'expJ' : mat_ccode(value['jordan']['expJ'],'expJ'),
        },
        'int' : {
          'vec' : mat_ccode(value['int']['vec'], 'vec'),
        },
      }
    else :
      ccode[key] = mat_ccode(value, key)

  print symbols, ccode
  return symbols, ccode

def generate_matrix(mA, mB) :
  r = sympy.symbols('r')
  n, m = len(mA), len(mB[0])
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
  return n, m, A, B, R

def generate_ccode_matrix(n, m, mat, prefix) :
  mat_str = prefix
  for i in range(n) :
    for j in range(m) :
      mat_str = mat_str + sympy.ccode(mat[i,j])
      if ((i+1)*(j+1) < max(n*m-1,2)) :
        mat_str = mat_str + ', '
  mat_str = mat_str + ';'
  return mat_str

def generate_cost_function(A, xf, B, R, xbar, solve_gramian_inv=True) :
  t, tau = sympy.symbols('t tau')
  G = sympy.integrate(sympy.exp(A*(t-tau))*B*R.inv()*B.transpose()*sympy.exp(A.transpose()*(t-tau)), (tau, 0, t))
  print 'G : {}'.format(G)
  if solve_gramian_inv :
    d = G.inv() * (xf - xbar)
    t1 = (2*A*xf).transpose()*d
    t2 = d.transpose()*B*R.inv()*B.transpose()*d
    c = sympy.Matrix([t]) + (xf -xbar).transpose() * G.inv() * (xf - xbar)
    dc = sympy.Matrix([1])- t1 - t2
  else : 
    ## inverse of gramian unresolved
    g_inv_sym = sympy.symbols(', '.join(['g{}'.format(i) for i in range(G.rows*G.cols)]))
    # print g_inv_sym
    print 'creating symbols for the inverse of gramian...'
    g_inv = sympy.Matrix(G.rows,G.cols,list(g_inv_sym))
    print 'creating symbols for d vector...'
    d_symbols = generate_symbol(A.rows, 'd{}')
    print 'creating symbolic d vector...'
    d_sym = sympy.Matrix(A.rows,1,d_symbols)
    # print g_inv
    print 'computing d vector...'
    d = g_inv * (xf - xbar)
    # print d
    print 'computing the first term of the derivative of the cost fn...'
    # t1 = (2*A*xf).transpose()*d
    t1 = (2*A*xf).transpose()*d_sym
    # print t1
    print 'computing the second term of the derivative of the cost fn...'
    # t2 = d.transpose()*B*R.inv()*B.transpose()*d
    t2 = d_sym.transpose()*B*R.inv()*B.transpose()*d_sym
    # print t2
    # print 'computing the cost function...'
    # c = sympy.Matrix([t]) + (xf - xbar).transpose() * g_inv * (xf - xbar)
    # print 'generating symbols for cost function'
    # xf_sym, xbar_sym, ginv = generate_symbol('xf xbar ginv')
    # c = sympy.sympify('Matrix(t+(xf_sym-xbar_sym).transpose() * ginv * (xf_sym-xbar_sym))')
    ## resolve cost function at code genration!
    c = 't + (xf-xbar).transpose() * ginv * (xf-xbar)'
    # print c
    print 'computing the derivative of the cost function...'
    dc = '1+2*(A*xf+c).transpose()*d-d.transpose()*B*R.inverse()*B.transpose()*d'
    d = d_sym
    # print dc
  return t1, t2, c, dc, d, G

def generate_controller(mA, mB) :
  r, t, b, tau = sympy.symbols('r t b tau')
  #r, t, po, vo, pf, vf, b, tau = sympy.symbols('r t po vo pf vf b tau')

  n, m, A, B, R = generate_matrix(mA, mB)

  cmp_mat = (A.row_join(B*R.inv()*B.transpose())).col_join(sympy.Matrix.zeros(A.rows,A.cols).row_join(-A.transpose()))
  exp_cmp_mat = sympy.exp(cmp_mat*t)
  print R, '\n', cmp_mat
  [cmp_P, cmp_J] = cmp_mat.jordan_form()

  x = []
  for i in range(n) :
    x.append(sympy.symbols('x%s'%i))

  xi_list, xf_list, if_var_str, if_set_str = generate_init_final_state(n)

  # xi = sympy.Matrix(xi_list)
  # xf = sympy.Matrix(xf_list)
  # xbar = sympy.exp(A*t)*xi

  xi, xf, xbar = generate_state_matrices(xi_list, xf_list, A)  
  t1, t2, c, dc, d, G = generate_cost_function(A, xf, B, R, xbar)

  def generate_aabb(xbar, G, b, t, n) :
    aabb, d_aabb = [], []
    for i in range(n) :
      aabb.append([xbar[i]-sympy.sqrt(G[i,i]*(b-t)), xbar[i]+sympy.sqrt(G[i,i]*(b-t))])
      d_aabb.append([sympy.diff(aabb[i][0],t), sympy.diff(aabb[i][1], t)])
    global generate_ccode_matrix
    aabb_str = generate_ccode_matrix(n,2,sympy.Matrix(aabb), 'aabb << ')
    d_aabb_str = generate_ccode_matrix(n,2,sympy.Matrix(d_aabb), 'd_aabb << ')
    return aabb, d_aabb, aabb_str, d_aabb_str

  aabb, d_aabb, aabb_str, d_aabb_str = generate_aabb(xbar, G, b, t, n)

  vr = sympy.Matrix([sympy.pi**2*(G*(b-t)).det()])
  eat = sympy.exp(A*t)
  [P, J] = A.jordan_form()

  C = sympy.Identity(n)

  def generate_cmp_str(exp_cmp_mat, cmp_mat, cmp_J, cmp_P) :
    global generate_ccode_matrix
    cmp_mat_str = generate_ccode_matrix(cmp_mat.rows, cmp_mat.cols, cmp_mat, 'A << ')
    exp_cmp_mat_str = generate_ccode_matrix(exp_cmp_mat.rows, exp_cmp_mat.cols, exp_cmp_mat, 'eAt << ')
    cmp_P_str = generate_ccode_matrix(cmp_P.rows, cmp_P.cols, cmp_P, 'P << ')
    cmp_J_str = generate_ccode_matrix(cmp_J.rows, cmp_J.cols, cmp_J, 'J << ')
    return cmp_J_str, cmp_P_str, exp_cmp_mat_str, cmp_mat_str
  
  cmp_J_str, cmp_P_str, exp_cmp_mat_str, cmp_mat_str = generate_cmp_str(exp_cmp_mat, cmp_mat, cmp_J, cmp_P)

  def generate_ccode_matrix(n, A, B, C, G, P, J, eat) :
    global generate_ccode_matrix
    gram_str = generate_ccode_matrix(n, n, G, 'G << ')
    eat_str = generate_ccode_matrix(n, n, eat, 'eAt << ')
    p_str = generate_ccode_matrix(n, n, P, 'P << ')
    j_str = generate_ccode_matrix(n, n, J, 'J << ')
    a_str = generate_ccode_matrix(n, n, A, 'A << ')
    b_str = generate_ccode_matrix(B.rows, B.cols, B, 'B << ')
    c_str = generate_ccode_matrix(n, n, C, 'C << ')
    jordan_str = p_str + '\n' + j_str
    return a_str, b_str, c_str, p_str, j_str, eat_str, jordan_str, gram_str

  a_str, b_str, c_str, p_str, j_str, eat_str, jordan_str, gram_str = generate_ccode_matrix(n, A, B, C, G, P, J, eat)

  cost_str = 'cost = %s;' % sympy.ccode(c[0])
  dc_str = 'd_cost = %s;' % sympy.ccode(dc[0])
  vr_str = 'vr = %s;' % sympy.ccode(vr[0])
  
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
