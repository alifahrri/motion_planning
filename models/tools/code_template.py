#!/usr/bin/env python
# -*- coding: utf-8 -*-

def generate_nonlinear_cpp(model_name, 
    dim, u_dim,
    n, m, A, B, R, d, f, c, P, J, expJ,
    x_bar, 
    gramian,
    solution, 
    cost,
    d_cost
  ) :
  print 'generating nonlinear cpp code', model_name
  linearization_variables = ''.join(['auto &x{}_hat = state({}); '.format(i+1, i) for i in range(int(dim))])
  linearization_expr = '{}\n{}\n{}\n{}\n'.format(
    linearization_variables,
    P, J, 'P_inv = P.inverse();'
  )
  expm_jordan_expr = '{}\n{}\n'.format(
    linearization_variables,
    expJ
  )
  ppinv_jordan_expr = '{}\n{}\n'.format(
    P, 'P_inv = P.inverse();'
  )
  constant_expr = '{}\n{}\n'.format(
    linearization_variables,
    c
  )
  cmp_jordan_expr = '{}\n{}\n'.format(
    solution['jordan']['P'],
    solution['jordan']['J'],
  )
  expm_cmp_jordan_expr = '{}\n{}\n'.format(
    linearization_variables,
    solution['jordan']['expJ'], 
  )
  ppinv_cmp_jordan_expr = '{}\n{}\n'.format(
    solution['jordan']['P'],
    'P_inv = P.inverse();',
  )
  # integration term for the derivative of the cost
  integration_term_expr = '{};\n'.format(
    'integration_term << ' + ','.join(['integrator.eval<{}>(0,t)'.format(i) for i in range(dim)])
  )
  code_str = str(
    '''
    // Warning : this code is generated automatically, any changes might be overwritten
    #ifndef {0}_HPP
    #define {0}_HPP

    #include "statespace.hpp"
    #include "statespacesolver.hpp"
    #include "fixedtimelqr.hpp"

    #include <memory>

    namespace Models {{

    #define SYS_N {2}
    #define SYS_P {3}
    #define SYS_Q {2}

    constexpr int n = SYS_N;
    constexpr int p = SYS_P;
    constexpr int q = SYS_Q;

    typedef double Scalar;
    constexpr Scalar r = 1.0;

    /* unresolved, use jordan form instead
    struct {0}ClosedExpm
    {{
      Eigen::Matrix<Scalar,SYS_N,SYS_N> operator()(Scalar t) const
      {{
        Eigen::Matrix<Scalar,SYS_N,SYS_N> eAt;
        {{4}}
        return eAt;
      }}
      Scalar r = Models::r;
    }};
    */
    /*
    struct {0}JordanForm
    {{
      typedef std::tuple<{0}SS::SystemMatrix,{0}SS::SystemMatrix> Mat;
      {0}SS::SystemMatrix J, P;
      {0}JordanForm()
      {{
        {{5}}
      }}
      Mat operator()() {{
        return std::make_tuple(J,P);
      }}
    }};
    */

    /* compute eAt using jordan form */
    struct {0}JordanFormExpm
    {{
      Eigen::Matrix<Scalar,SYS_N,SYS_N> operator()(Scalar t) const
      {{
        Eigen::Matrix<Scalar,SYS_N,SYS_N> eAt, expJ;
				auto &state = x_hat;
        {7}
        eAt = P*expJ*P_inv;
        return eAt;
      }}
			Eigen::Matrix<Scalar,SYS_N,SYS_N> operator()(const Eigen::Matrix<Scalar,SYS_N,1> &state, Scalar t)
      {{
        Eigen::Matrix<Scalar,SYS_N,SYS_N> eAt, expJ;
        {7}
        {13}
        eAt = P*expJ*P_inv;
        return eAt;
      }}
      void linearize(const Eigen::Matrix<Scalar,SYS_N,1> &state)
      {{
        x_hat = state;
        {12}
        {13}
      }}
      Scalar r = Models::r;
      /* linearization state */
      Eigen::Matrix<Scalar,SYS_N,1> x_hat;
      Eigen::Matrix<Scalar,SYS_N,SYS_N> P, P_inv;
    }};

    /* unresolved, use jordan form instead
    typedef StateSpace<Scalar,SYS_N,SYS_P,SYS_Q,{0}ClosedExpm> {0}SS;
    typedef StateSpaceSolver<Scalar,SYS_N,SYS_P,SYS_Q,{0}SS> {0}Solver;
    */

    typedef StateSpace<Scalar,SYS_N,SYS_P,SYS_Q,{0}JordanFormExpm> {0}SS;
    typedef StateSpaceSolver<Scalar,SYS_N,SYS_P,SYS_Q,{0}SS> {0}Solver;

    {0}SS {1};
    {0}Solver {1}_solver({1});

    struct {0}Linearization
    {{
      typedef std::tuple<
        Eigen::Matrix<Scalar,SYS_N,SYS_N>,
        Eigen::Matrix<Scalar,SYS_N,SYS_N>,
        Eigen::Matrix<Scalar,SYS_N,SYS_N>
      > LinearizationMat;

      /* linearize matrix and create jordan form */
      LinearizationMat operator()(const Eigen::Matrix<Scalar,SYS_N,1> &state) const
      {{
        Eigen::Matrix<Scalar,SYS_N,SYS_N> P, J, P_inv;
        {4}
        return std::make_tuple(P, J, P_inv);
      }}
      /* linearize system matrix only */
			/* TODO : resolve return type
      std::tuple<Eigen::Matrix<Scalar,SYS_N,SYS_N>> operator()(const Eigen::Matrix<Scalar,SYS_N,1> &state) const
      {{

      }}
			*/
      Scalar r = Models::r;
    }};

    struct {0}LinearizationConstant 
    {{
      void linearize(const Eigen::Matrix<Scalar,SYS_N,1> &state) 
      {{
        {6}
      }}
      Eigen::Matrix<Scalar,SYS_N,1>& operator()(const Eigen::Matrix<Scalar,SYS_N,1> &state)
      {{
        {6}
        return c;
      }}
      const Eigen::Matrix<Scalar,SYS_N,1>& operator()() const
      {{
        return c;
      }}
      Eigen::Matrix<Scalar,SYS_N,1> c;
    }};

    struct {0}Cost
    {{
      template <typename GramianFn>
      auto operator()(const {0}SS &system, const {0}SS::StateType &xi, const {0}SS::StateType &xf, const {0}SS::StateType &c, const Scalar &t, GramianFn &g) const
        -> decltype(g(t).inverse(), Scalar())
      {{
				const auto& sys = system;
        auto f = [&](Scalar t) {{
          return sys.expm(t)*c;
        }};
        constexpr int max_depth = 100;
        constexpr Scalar e = Scalar(1e-3);
        using integration_t = decltype(f);
        SimpsonsRuleIntegrator<Scalar,integration_t,max_depth> integrator(f,e);
        auto xbar = exp(t)*xi + integrator(0,t);
        /* todo : execute around pointer instead of dereferencing */
        return (*this)(xbar, xf, t, g(t).inverse());
      }}
      auto operator()(const {0}SS::StateType &xbar, const {0}SS::StateType &xf, const Scalar &t, const {0}SS::SystemMatrix &ginv) const
        -> decltype(t+(xf-xbar).transpose()*ginv*(xf-xbar))
      {{
        Scalar cost;
        cost = t+(xf-xbar).transpose()*ginv*(xf-xbar);
        return cost;
      }}
      /* input weighting factor */
      Scalar r = Models::r;
      /* a pointer to gramian, the implementation should set the appropriate address befor call the functor */
      // std::shared_ptr<{0}Gramian> g;
    }};

    // {0}Cost {1}_cost;

    /* evaluate mat operation term of xbar integration */
    struct {0}IntegratorEval
    {{
      {0}IntegratorEval(const {0}SS &sys, const {0}SS::StateType &c)
        : sys(sys), c(c)
      {{}}
      template <size_t i>
      Scalar eval(Scalar t0, Scalar t1)
      {{
        {0}SS::StateType result;
        auto f = [&](Scalar t) {{
          return (sys.expm(t)*c)[i];
        }};
        constexpr int max_depth = 100;
        constexpr Scalar e = Scalar(1e-3);
        using integration_t = decltype(f);
        SimpsonsRuleIntegrator<Scalar,integration_t,max_depth> integrator(f,e);
        return integrator(t0,t1);
      }}
      const {0}SS &sys;
      const {0}SS::StateType &c;
    }};

    struct {0}Gramian
    {{
			void linearize(const {0}SS::StateType &state) 
			{{
					this->state = state;
			}}
			{0}SS::SystemMatrix operator()(const {0}SS::StateType &state, Scalar t)
      {{
				linearize(state);
				{12}
        {0}SS::SystemMatrix gramian;
        {10}
        return gramian;
      }}
      {0}SS::SystemMatrix operator()(Scalar t) const
      {{
				{12}
        {0}SS::SystemMatrix gramian;
        {10}
        return gramian;
      }}
			{0}SS::StateType state;
      Scalar r = Models::r;
    }};

    /* TODO : rename to cost diff or cost derivative to avoid confusion */
    struct {0}OptTimeDiff
    {{
      {0}OptTimeDiff(const {0}SS &system, const {0}LinearizationConstant &cconstant, const {0}Gramian &gramian)
        : system(system), cconstant(cconstant), g(gramian)
      {{}}
      auto set(const {0}SS::StateType &xi, const {0}SS::StateType &xf)
      {{
        this->xi = xi;
        this->xf = xf;
      }}
      Scalar operator()(const Scalar t) const
      {{
				using cconst_t = decltype(cconstant());
        using d_vect_t = decltype(xf);
				const auto& sys = system;
        cconst_t c = cconstant();
        {0}IntegratorEval integrator(sys, c);
        {0}SS::StateType integration_term;
        /* integrate each element of the integration term */
        {15}
        auto xbar = sys.expm(t)*xi + integration_term;
        d_vect_t d = g(t).inverse()*(xf-xbar);
        /* todo : execute around pointer instead of dereferencing */
        return (*this)(xf, d, c, g(t).inverse());
      }}
			Scalar operator()(const {0}SS::StateType &xf, const {0}SS::StateType &d, const {0}SS::StateType &c, const {0}SS::SystemMatrix &ginv) const
      {{
        Eigen::Matrix<Scalar,SYS_P,SYS_P> R;
        R = decltype(R)::Identity() * r;
        const auto& A = system.A;
        const auto& B = system.B;
        Scalar d_cost;
        d_cost = 1+2*(A*xf+c).transpose()*d-d.transpose()*B*R.inverse()*B.transpose()*d;
        return d_cost;
      }}

      Scalar r = Models::r;
      {0}SS::StateType xi, xf;
      const {0}SS &system;
      const {0}LinearizationConstant &cconstant;
      const {0}Gramian &g;
    }};

    // {0}OptTimeDiff {1}_opt_time_diff;

    // {0}Gramian {1}_gram;

    /* unresolved, use jordan form instead
    struct {0}CmpClosedExpm
    {{
      Eigen::Matrix<Scalar,2*SYS_N,2*SYS_N> operator()(Scalar t) const
      {{
        Eigen::Matrix<Scalar,2*SYS_N,2*SYS_N> eAt;        
        return eAt;
      }}
      Scalar r = Models::r;
    }};
    */

    /* compute eAt (of the composite system) using jordan form */
    struct {0}CmpJordanFormExpm
    {{
      Eigen::Matrix<Scalar,2*SYS_N,2*SYS_N> operator()(Scalar t) const
      {{
        Eigen::Matrix<Scalar,2*SYS_N,2*SYS_N> eAt, expJ;
        {11}
        eAt = P*expJ*P_inv;
        return eAt;
      }}
      void linearize(const Eigen::Matrix<Scalar,SYS_N,1> &state)
      {{
        this->state = state;
        {12}
        {14}
      }}
      Scalar r = Models::r;
      /* linearization state */
      Eigen::Matrix<Scalar,SYS_N,1> state;
      Eigen::Matrix<Scalar,2*SYS_N,2*SYS_N> P, P_inv;
    }};

    /*
    struct {0}CmpJordanForm
    {{
      typedef std::tuple<{0}SSComposite::SystemMatrix,{0}SSComposite::SystemMatrix> Mat;
      {0}SS::SystemMatrix J, P;
      {0}CmpJordanForm()
      {{
        {{11}}
      }}
      Mat operator()() {{
        return std::make_tuple(J,P);
      }}
    }};
    */

    /* unresolved, use jordan form instead
    typedef StateSpace<Scalar,2*SYS_N,SYS_P,SYS_Q,{0}CmpClosedExpm> {0}SSComposite;
    */
    typedef StateSpace<Scalar,2*SYS_N,SYS_P,SYS_Q,{0}CmpJordanFormExpm> {0}SSComposite;
    typedef OptimalTimeFinder<{0}OptTimeDiff> {0}OptTimeSolver;

    /* TODO : fix
    typedef FixedTimeLQR<{0}SS,{0}Gramian> {0}FixTimeLQR;
    typedef OptTrjSolver<{0}Cost,{0}OptTimeSolver,{0}FixTimeLQR,{0}SS,{0}Gramian,{0}SSComposite> {0}TrajectorySolver;
    */

    // {0}SSComposite {1}_ss_cmp;
    // {0}FixTimeLQR {1}_ft_lqr({1},{1},{1}_gram);
    // {0}OptTimeSolver {1}_opt_time_solver({1}_opt_time_diff);
    // {0}TrajectorySolver {1}_trj_solver({1}_cost, {1}_opt_time_solver, {1}_ft_lqr,{1},{1}_gram,{1}_ss_cmp);

    /* TODO : fix */
    struct {0} {{
      typedef {0}SS::StateType State;
      typedef {0}SS::StateType Input;

      {0}()
        : opt_time_diff(state_space, cconstant, gramian)
        , opt_time_solver(opt_time_diff)
      {{ }}

      {0}SS state_space;
      {0}Cost cost;
      {0}Gramian gramian;
      {0}SSComposite composite_ss;
      {0}LinearizationConstant cconstant;
      {0}OptTimeDiff opt_time_diff;
      {0}OptTimeSolver opt_time_solver;
      // {0}FixTimeLQR ft_lqr = {0}FixTimeLQR(state_space, state_space, gramian);
      // {0}TrajectorySolver solver = {0}TrajectorySolver(cost, opt_time_solver, ft_lqr, state_space, gramian, composite_ss);

      void set_weight(Scalar r) {{
        cost.r = r;
        gramian.r = r;
        opt_time_diff.r = r;
        // using RMat = decltype(ft_lqr.R);
        // ft_lqr.R = RMat::Identity()*r;
        state_space.exp_fn.r = r;
        composite_ss.exp_fn.r = r;
      }}

      void linearize(const State &state) {{
        state_space.linearize(state);
        gramian.linearize(state);
        composite_ss.linearize(state);
        cconstant.linearize(state);
      }}
    }};

    }} // namespace model

    #endif // MODELS_HPP
    '''
  ).format(
    model_name,           #0
    model_name.lower(),   #1
    dim,                  #2
    u_dim,                #3
    linearization_expr,   #4
    cmp_jordan_expr,      #5
    constant_expr,        #6
    expm_jordan_expr,     #7
    '',                   #8
    d_cost,               #9
    gramian,              #10
    expm_cmp_jordan_expr, #11
		linearization_variables, #12
    ppinv_jordan_expr,    #13
    ppinv_cmp_jordan_expr,#14
    integration_term_expr,#15
    '', '',
    '', '', '', '', '', '',
    '', '', '', '',
  )
  print 'generating nonlinear cpp code', model_name, 'OK'
  return code_str

def generate_nonlinear_test_cpp(model_name, dim) :
  print 'generating test code for model : ', model_name, '...'
  test_linearization_state = 'Eigen::Matrix<double,{0},1> state; \nstate << {1};'.format(
    dim,
    ','.join(['1']*5)
  )
  test_initial_state = 'Eigen::Matrix<double,{0},1> init_state; \ninit_state << {1};'.format(
    dim,
    ','.join(['0']*5)
  )
  test_code = str(
    '''
    #include <gtest/gtest.h>
    #include "{1}.hpp"

    typedef Models::{0}OptTimeDiff {0}TimeDiff;
    // typedef Models::{0}OptTimeSolver {0}TimeSolver;
    // typedef Models::{0}TrajectorySolver {0}TrajectorySolver;
    typedef Models::{0}SS {0}SS;
    typedef Models::{0}Gramian {0}Gramian;
    typedef Models::{0}JordanFormExpm {0}ClosedExpm;
    typedef Models::{0}SSComposite {0}SSComposite;
    typedef Models::{0}CmpJordanFormExpm {0}CmpClosedExpm;
    typedef Models::{0}SSComposite::StateType {0}SSCompositeState;
    typedef Models::{0}SSComposite::SystemMatrix {0}SSCompositeSystem;

    TEST({0}TimeSolver, d_cost_near_zero) {{
      Models::{0} {1};
      auto &time_diff = {1}.opt_time_diff;
      auto &time_solver = {1}.opt_time_solver;

      /* initial state */
      {7}
      /* final state */
      {6}
      /* linearization */
      {1}.linearize(state);

      auto opt_time = time_solver.solve(init_state,state);
      time_diff.set(init_state,state);
      auto d_cost = time_diff(opt_time);
      EXPECT_NEAR(d_cost, 0.0, 1e-4) << d_cost;
    }}

    TEST({0}Gramian, gram_no_inf_nan) {{
      {0}Gramian g;
      auto ok = true;
      std::stringstream ss;
      /* linearization */
      {6}
      g.linearize(state);
      for(size_t i=1; i<30; i++) {{
        auto t = i*0.5;
        auto m = g(t);
        auto m_inv = m.inverse();
        ss << "t(" << t << ") : [";
        for(size_t j=0; j<{4}; j++) {{
          for(size_t k=0; k<{4}; k++) {{
            ss << m(j,k) << (k!=({4}-1) ? " " : "; ");
            if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
          }}
        }}
        ss << "]" << std::endl;
        ss << "t(" << t << ") : inverse [";
        for(size_t j=0; j<{4}; j++) {{
          for(size_t k=0; k<{4}; k++) {{
            ss << m_inv(j,k) << (k!=({4}-1) ? " " : "; ");
            if(isnan(m_inv(j,k)) || isinf(m_inv(j,k))) ok = false;
          }}
        }}
        ss << "]" << std::endl;
      }}
    }}

    TEST({0}ClosedExpm, exp_no_inf_nan) {{
			{0}ClosedExpm {1}_exp;
			auto ok = true;
			std::stringstream ss;
      /* linearization */
      {6}
      {1}_exp.linearize(state);
			for(size_t i=0; i<10; i++) {{
				auto t = i*0.5;
				auto m = {1}_exp(t);
				ss << "t(" << t << ") : [";
				for(size_t j=0; j<{4}; j++) {{
					for(size_t k=0; k<{4}; k++) {{
						ss << m(j,k) << (k!=({4}-1) ? " " : "; ");
						if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
					}}
				}}
				ss << "] " << std::endl;
			}}
			EXPECT_TRUE(ok) << ss.str();
	  }}

    TEST({0}SS, exp_no_inf_nan) {{
      Models::{0} {1};
      auto &{1}_ss = {1}.state_space;
      auto ok = true;
      std::stringstream ss;
      /* linearization */
      {6}
      {1}_ss.linearize(state);
      for(size_t i=0; i<10; i++) {{
        auto t = i*0.5;
        auto m = {1}_ss.expm(t);
        ss << "t(" << t << ") : [";
        for(size_t j=0; j<{4}; j++) {{
          for(size_t k=0; k<{4}; k++) {{
            ss << m(j,k) << (k!=({4}-1) ? " " : "; ");
            if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
          }}
        }}
        ss << "]" << std::endl;
      }}
      EXPECT_TRUE(ok) << ss.str();
    }}

    TEST({0}CmpClosedExpm, exp_no_inf_nan) {{
			{0}CmpClosedExpm {1}_exp;
			auto ok = true;
			std::stringstream ss;
      /* linearization */
      {6}
      {1}_exp.linearize(state);
			for(size_t i=0; i<10; i++) {{
				auto t = i*0.5;
				auto m = {1}_exp(t);
				ss << "t(" << t << ") : [";
				for(size_t j=0; j<{5}; j++) {{
					for(size_t k=0; k<{5}; k++) {{
						ss << m(j,k) << (k!=({4}-1) ? " " : "; ");
						if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
					}}
				}}
				ss << "] " << std::endl;
			}}
			EXPECT_TRUE(ok) << ss.str();
	  }}

    TEST({0}SSComposite, exp_no_inf_nan) {{
      Models::{0} {1};
      auto &{1}_ss = {1}.composite_ss;
      auto ok = true;
      std::stringstream ss;
      /* linearization */
      {6}
      {1}_ss.linearize(state);
      for(size_t i=0; i<10; i++) {{
        auto t = i*0.5;
        auto m = {1}_ss.expm(t);
        ss << "t(" << t << ") : [";
        for(size_t j=0; j<{5}; j++) {{
          for(size_t k=0; k<{5}; k++) {{
            ss << m(j,k) << (k!=({5}-1) ? " " : "; ");
            if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
          }}
        }}
        ss << "]" << std::endl;
      }}
      EXPECT_TRUE(ok) << ss.str();
    }}
    '''.format(
      model_name,
      model_name.lower(),
      ', '.join(['1.0' for i in range(dim)]),
      ', '.join(['0.0' for i in range(dim)]),
      dim,
      dim*2,
      test_linearization_state,
      test_initial_state
    )
  )
  print 'generating test code for model : ', model_name, '...OK'
  return test_code

def generate_cpp(model_name, dim, u_dim, g, jordan, eat, c, dc, aabb, d_aabb, vr, if_var, if_set, a_str, b_str, c_str, cmp_J_str, cmp_P_str, cmp_exp) :
  print(
    model_name,           #{0}
    model_name.lower(),   #{1}
    dim,                  #{2}
    u_dim,                #{3}
    eat,                  #{4}
    jordan,               #{5}
    if_var,               #{6}
    if_set,               #{7}
    c,                    #{8}
    dc,                   #{9}
    g,                    #{10}
    cmp_exp,              #{11}
    a_str,                #{12}
    b_str,                #{13}
    c_str,                #{14}
    cmp_P_str,            #{15}
    cmp_J_str             #{16}
  )
  code_str = str(
  '''
  // Warning : this code is generated automatically, any changes might be overwritten
  #ifndef {0}_HPP
  #define {0}_HPP

  #include "statespace.hpp"
  #include "statespacesolver.hpp"
  #include "fixedtimelqr.hpp"

  namespace Models {{

  #define SYS_N {2}
  #define SYS_P {3}
  #define SYS_Q {2}

  constexpr int n = SYS_N;
  constexpr int p = SYS_P;
  constexpr int q = SYS_Q;

  typedef double Scalar;
  constexpr Scalar r = 1.0;

  struct {0}ClosedExpm
  {{
    Eigen::Matrix<Scalar,SYS_N,SYS_N> operator()(Scalar t) const
    {{
      Eigen::Matrix<Scalar,SYS_N,SYS_N> eAt;
      {4}
      return eAt;
    }}
		Scalar r = Models::r;
  }};

  typedef StateSpace<Scalar,SYS_N,SYS_P,SYS_Q,{0}ClosedExpm> {0}SS;
  typedef StateSpaceSolver<Scalar,SYS_N,SYS_P,SYS_Q,{0}SS> {0}Solver;
  struct {0}JordanForm
  {{
    typedef std::tuple<{0}SS::SystemMatrix,{0}SS::SystemMatrix> Mat;
    {0}SS::SystemMatrix J, P;
    {0}JordanForm()
    {{
      {5}
    }}
    Mat operator()() {{
      return std::make_tuple(J,P);
    }}
  }};

  {0}SS {1};
  {0}Solver {1}_solver({1});

  struct {0}Cost
  {{
    Scalar operator()(const {0}SS::StateType &xi, const {0}SS::StateType &xf, const Scalar &t) const
    {{
      Scalar cost;
      {6}
      {7}
      {8}
      return cost;
    }}
		Scalar r = Models::r;
  }};

  {0}Cost {1}_cost;

  struct {0}OptTimeDiff
  {{
    void set(const {0}SS::StateType &xi, const {0}SS::StateType &xf)
    {{
      {7}
    }}
    Scalar operator()(const Scalar &t) const
    {{
      Scalar d_cost;
      {9}
      return d_cost;
    }}

    {6}
		Scalar r = Models::r;
  }};

  {0}OptTimeDiff {1}_opt_time_diff;

  struct {0}Gramian
  {{
    {0}SS::SystemMatrix operator()(Scalar t) const
    {{
      {0}SS::SystemMatrix G;
      {10}
      return G;
    }}
		Scalar r = Models::r;
  }};

  {0}Gramian {1}_gram;

  struct {0}CmpClosedExpm
  {{
    Eigen::Matrix<Scalar,2*SYS_N,2*SYS_N> operator()(Scalar t) const
    {{
      Eigen::Matrix<Scalar,2*SYS_N,2*SYS_N> eAt;
      {11}
      return eAt;
    }}
		Scalar r = Models::r;
  }};

  typedef StateSpace<Scalar,2*SYS_N,SYS_P,SYS_Q,{0}CmpClosedExpm> {0}SSComposite;
  typedef FixedTimeLQR<{0}SS,{0}Gramian> {0}FixTimeLQR;
  typedef OptimalTimeFinder<{0}OptTimeDiff> {0}OptTimeSolver;
  typedef OptTrjSolver<{0}Cost,{0}OptTimeSolver,{0}FixTimeLQR,{0}SS,{0}Gramian,{0}SSComposite> {0}TrajectorySolver;

  {0}SSComposite {1}_ss_cmp;
  {0}FixTimeLQR {1}_ft_lqr({1},{1},{1}_gram);
  {0}OptTimeSolver {1}_opt_time_solver({1}_opt_time_diff);
  {0}TrajectorySolver {1}_trj_solver({1}_cost, {1}_opt_time_solver, {1}_ft_lqr,{1},{1}_gram,{1}_ss_cmp);

  struct {0} {{
    typedef {0}SS::StateType State;
    typedef {0}SS::StateType Input;

    {0}()
    {{
      auto &ss = this->state_space;
      ss.{12}
      ss.{13}
      ss.{14}
      // jordan form should be depecrated, use closed form of exp mat instead
      // TODO : remove
      {0}JordanForm {1}_jordan_form;
      auto t = {1}_jordan_form();
      ss.D = std::get<0>(t);
      ss.P = std::get<1>(t);
      ss.P_inv = ss.P.inverse();

      auto R = this->ft_lqr.R;
      auto &ss_cmp = this->composite_ss;
      ss_cmp.A << ss.A, ss.B*R.inverse()*ss.B.transpose(), {0}SS::SystemMatrix::Zero(), -ss.A.transpose();
      ss_cmp.{15}
      ss_cmp.{16}
      ss_cmp.P_inv = ss_cmp.P.inverse();
    }}

    {0}SS state_space;
    {0}Cost cost;
    {0}Gramian gramian;
    {0}SSComposite composite_ss;
    {0}OptTimeDiff opt_time_diff;
    {0}FixTimeLQR ft_lqr = {0}FixTimeLQR(state_space, state_space, gramian);
    {0}OptTimeSolver opt_time_solver = {0}OptTimeSolver(opt_time_diff);
    {0}TrajectorySolver solver = {0}TrajectorySolver(cost, opt_time_solver, ft_lqr, state_space, gramian, composite_ss);

    // support for changing input weight at runtime;
    void set_weight(Scalar r) {{
      cost.r = r;
      gramian.r = r;
      opt_time_diff.r = r;
      using RMat = decltype(ft_lqr.R);
      ft_lqr.R = RMat::Identity()*r;
      state_space.exp_fn.r = r;
      composite_ss.exp_fn.r = r;
    }}
  }};

  // [[deprecated(use {0} class instead)]]
  void init_{1}()
  {{
    auto &ss = {1};
    {0}JordanForm {1}_jordan_form;
    ss.{12}
    ss.{13}
    ss.{14}
    auto t = {1}_jordan_form();
    ss.D = std::get<0>(t);
    ss.P = std::get<1>(t);
    ss.P_inv = ss.P.inverse();

    auto R = {1}_ft_lqr.R;
    auto &ss_cmp = {1}_ss_cmp;
    ss_cmp.A << ss.A, ss.B*R.inverse()*ss.B.transpose(), {0}SS::SystemMatrix::Zero(), -ss.A.transpose();
    ss_cmp.{15}
    ss_cmp.{16}
    ss_cmp.P_inv = ss_cmp.P.inverse();
  }}

  // [[deprecated(use {0} class instead)]]
  {0}SS get_{1}()
  {{
    return {1};
  }}

  }} // namespace model

  #endif // MODELS_HPP
  '''.format(
    model_name,           #{0}
    model_name.lower(),   #{1}
    dim,                  #{2}
    u_dim,                #{3}
    eat,                  #{4}
    jordan,               #{5}
    if_var,               #{6}
    if_set,               #{7}
    c,                    #{8}
    dc,                   #{9}
    g,                    #{10}
    cmp_exp,              #{11}
    a_str,                #{12}
    b_str,                #{13}
    c_str,                #{14}
    cmp_P_str,            #{15}
    cmp_J_str             #{16}
  )
  )
  return code_str

def generate_cpp_main(model_name) :
  src = (
  '// Warning : this code is generated automatically, any changes might be overwritten\n'
  '#include <iostream>\n'
  '#include <fstream>\n'
  '\n'
  '//#define TEST\n'
  '#include "kronecker.hpp"\n'
  '#include "lyapunovsolver.hpp"\n'
  '#include "statespacesolver.hpp"\n'
  '#include "pythonembedded.hpp"\n'
  '#include "jordan.hpp"\n'
  '#include "lqrsolver.hpp"\n'
  '#include "%s.hpp"\n' % model_name.lower() +
  '#include "polynomial.hpp"\n'
  '\n'
  '//#define FEEDBACKCONTROLLER_DEBUG_TIME\n'
  '//#define FEEDBACKCONTROLLER_DEBUG_PRINT\n'
  '//#define FEEDBACKCONTROLLER_DEBUG_LOG\n'
  '#include "feedbackcontroller.hpp"\n'
  '\n'
  '#include <QApplication>\n'
  '#include "mainwindow.h"\n'
  '\n'
  'int main(int argc, char** argv)\n'
  '{\n'
  '  QApplication a(argc, argv);\n'
  '  Models::init_%s();\n' % model_name.lower() +
  '  auto %s = Models::get_%s();\n' % (model_name.lower(), model_name.lower()) +
  '  auto &%sft_lqr = Models::%s_ft_lqr;\n' % (model_name.lower(), model_name.lower()) +
  '  // auto xf = Models::%sSS::StateType::Identity();\n' % (model_name) +
  '  // auto x0 = Models::%sSS::StateType::Zero();\n' % (model_name) +
  '  auto& opt_trajectory = Models::%s_trj_solver;\n' % model_name.lower() +
  '\n'
  '  MainWindow w(Models::n, Models::p, MainWindow::FREE_FINAL_TIME);\n'
  '  w.readSystem<double,Models::n>(%s.A);\n'% model_name.lower() +
  '\n'
  '  // callback for w.set_initial push button\n'
  '  w.init_cb = [&](\n'
  '      std::vector<double> values,\n'
  '      std::vector<std::vector<double>>,\n'
  '      std::vector<double>)\n'
  '  {\n'
  '    std::vector<double> xi_vec, xf_vec;\n'
  '    std::vector<Models::%sSS::StateType> trj;\n' % model_name +
  '    std::vector<Models::%sSS::InputType> inputs;\n' % model_name +
  '    std::vector<double> time;\n'
  '    xi_vec.insert(xi_vec.begin(),values.begin(),values.begin()+values.size()/2);\n'
  '    xf_vec.insert(xf_vec.begin(),values.begin()+values.size()/2,values.end());\n'
  '    Models::%sSS::StateType xi(xi_vec.data()), xf(xf_vec.data());\n' % model_name +
  '    auto trajectory = opt_trajectory.solve(xi,xf);\n'
  '    std::stringstream ss;\n'
  '    auto c = opt_trajectory.cost(xi,xf);\n'
  '    ss << "cost : " << std::get<1>(c) << ", time : " << std::get<0>(c);\n'
  '    w.log(ss.str());\n'
  '    for(const auto& t : trajectory) {\n'
  '      trj.push_back(std::get<1>(t));\n'
  '      time.push_back(std::get<0>(t));\n'
  '      inputs.push_back(std::get<2>(t));\n'
  '    }\n'
  '    w.plotStates<double,Models::n>(trj,time);\n'
  '    w.plotInputs<double,Models::p>(inputs,time);\n'
  '  };\n'
  '\n'
  '  // dont do anything\n'
  '  w.system_cb = [&](std::vector<double> sys_values) {\n'
  '    w.readSystem<double,Models::n>(%s.A);\n' % model_name.lower() +
  '  };\n'
  '\n'
  '  w.showMaximized();\n'
  '  return a.exec();\n'
  '}\n'
  )
  return src

def generate_test_cpp(model_name, dim, u_dim, g, jordan, eat, c, dc, aabb, d_aabb, vr, if_var, if_set, a_str, b_str, c_str, cmp_J_str, cmp_P_str, cmp_exp) :
  test_code = (
  """
  #include <gtest/gtest.h>
  #include "{1}.hpp"

  typedef Models::{0}OptTimeDiff {0}TimeDiff;
  typedef Models::{0}OptTimeSolver {0}TimeSolver;
  typedef Models::{0}TrajectorySolver {0}TrajectorySolver;
  typedef Models::{0}SS {0}SS;
  typedef Models::{0}Gramian {0}Gramian;
  typedef Models::{0}ClosedExpm {0}ClosedExpm;
  typedef Models::{0}SSComposite {0}SSComposite;
  typedef Models::{0}CmpClosedExpm {0}CmpClosedExpm;
  typedef Models::{0}SSComposite::StateType {0}SSCompositeState;
  typedef Models::{0}SSComposite::SystemMatrix {0}SSCompositeSystem;

  TEST({0}TimeSolver, d_cost_near_zero) {{
    Models::{0} {1};
    auto &time_diff = {1}.opt_time_diff;
    auto &time_solver = {1}.opt_time_solver;

    {0}SS::StateType s0, s1;
    s0 << {2};
    s1 << {3};

    auto opt_time = time_solver.solve(s0, s1);
    time_diff.set(s0,s1);
    auto d_cost = time_diff(opt_time);
    EXPECT_NEAR(d_cost, 0.0, 1e-4) << d_cost;
  }}
	
	TEST({0}ClosedExpm, exp_no_inf_nan) {{
			{0}ClosedExpm {1}_exp;
			auto ok = true;
			std::stringstream ss;
			for(size_t i=0; i<10; i++) {{
				auto t = i*0.5;
				auto m = {1}_exp(t);
				ss << "t(" << t << ") : [";
				for(size_t j=0; j<{4}; j++) {{
					for(size_t k=0; k<{4}; k++) {{
						ss << m(j,k) << (k!=({4}-1) ? " " : "; ");
						if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
					}}
				}}
				ss << "] " << std::endl;
			}}
			EXPECT_TRUE(ok) << ss.str();
	}}
	
	TEST({0}SS, exp_no_inf_nan) {{
		Models::{0} {1};
		auto &{1}_ss = {1}.state_space;
		auto ok = true;
		std::stringstream ss;
		for(size_t i=0; i<10; i++) {{
			auto t = i*0.5;
			auto m = {1}_ss.expm(t);
			ss << "t(" << t << ") : [";
			for(size_t j=0; j<{4}; j++) {{
				for(size_t k=0; k<{4}; k++) {{
					ss << m(j,k) << (k!=({4}-1) ? " " : "; ");
					if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
				}}
			}}
			ss << "]" << std::endl;
		}}
		EXPECT_TRUE(ok) << ss.str();
	}}
	
	TEST({0}SSComposite, exp_no_inf_nan) {{
		Models::{0} {1};
		auto &{1}_ss = {1}.composite_ss;
		auto ok = true;
		std::stringstream ss;
		for(size_t i=0; i<10; i++) {{
			auto t = i*0.5;
			auto m = {1}_ss.expm(t);
			ss << "t(" << t << ") : [";
			for(size_t j=0; j<{4}; j++) {{
				for(size_t k=0; k<{4}; k++) {{
					ss << m(j,k) << (k!=({4}-1) ? " " : "; ");
					if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
				}}
			}}
			ss << "]" << std::endl;
		}}
		EXPECT_TRUE(ok) << ss.str();
	}}
	
	TEST({0}Gramian, gram_no_inf_nan) {{
		{0}Gramian g;
		auto ok = true;
		std::stringstream ss;
		for(size_t i=1; i<30; i++) {{
			auto t = i*0.5;
			auto m = g(t);
			auto m_inv = m.inverse();
			ss << "t(" << t << ") : [";
			for(size_t j=0; j<{4}; j++) {{
				for(size_t k=0; k<{4}; k++) {{
					ss << m(j,k) << (k!=({4}-1) ? " " : "; ");
					if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
				}}
			}}
			ss << "]" << std::endl;
			ss << "t(" << t << ") : inverse [";
			for(size_t j=0; j<{4}; j++) {{
				for(size_t k=0; k<{4}; k++) {{
					ss << m_inv(j,k) << (k!=({4}-1) ? " " : "; ");
					if(isnan(m_inv(j,k)) || isinf(m_inv(j,k))) ok = false;
				}}
			}}
			ss << "]" << std::endl;
		}}
	}}
	
	TEST({0}CmpClosedExpm, exp_no_inf_nan) {{
		{0}CmpClosedExpm cmp_exp;
		auto ok = true;
		std::stringstream ss;
		for(size_t i=0; i<10; i++) {{
			auto t = i*0.5;
			auto m = cmp_exp(t);
			ss << "t(" << t << ") : [" ;
			for(size_t j=0; j<{5}; j++) {{
				for(size_t k=0; k<{5}; k++) {{
					ss << m(j,k) << (k==({5}-1) ? " " : "; ");
					if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
				}}
			}}
			ss << "]" << std::endl;
		}}
		EXPECT_TRUE(ok) << ss.str();
	}}
	
	TEST({0}SSCompositeState, composite_state_no_inf_nan) {{
		Models::{0} {1};
		auto &g = {1}.gramian;
		auto &tsolver = {1}.opt_time_solver;
		auto &{1}_ss = {1}.state_space;
		{0}SS::StateType s0, s1;
		s0 << {2};
		s1 << {3};
		auto opt_time = tsolver.solve(s0, s1);
		auto g_mat = g(opt_time);
		auto g_inv = g_mat.inverse();
		auto expm = {1}_ss.expm(opt_time);
		auto d_opt = g_inv*(s1-(expm*s0));
		{0}SSCompositeState cmp_state;
		cmp_state << s1, d_opt;
		auto ok = true;
		std::stringstream ss;
		ss << "opt_time(" << opt_time << ") : [";
		for(size_t k=0; k<{4}; k++) {{
			ss << d_opt(k) << (k==({4}-1) ? "" : " ");
			if(isnan(d_opt(k)) || isinf(d_opt(k))) ok = false;
		}}
		ss << "]" << std::endl;
		for(size_t j=0; j<{5}; j++) {{
			ss << cmp_state(j) << (j==({5}-1) ? "" : " ");
			if(isnan(cmp_state(j)) || (isinf(cmp_state(j)))) ok = false;
		}}
		ss << "]" << std::endl;
		EXPECT_TRUE(ok) << ss.str();
	}}
  """.format(
    model_name,
    model_name.lower(),
    ', '.join(['1.0' for i in range(dim)]),
    ', '.join(['0.0' for i in range(dim)]),
		dim,
		dim*2
  )
  )
  return test_code
