
    // Warning : this code is generated automatically, any changes might be overwritten
    #ifndef NonlinearCar_HPP
    #define NonlinearCar_HPP

    #include "statespace.hpp"
    #include "statespacesolver.hpp"
    #include "fixedtimelqr.hpp"

    #include <memory>

    namespace Models {

    #define SYS_N 5
    #define SYS_P 2
    #define SYS_Q 5

    constexpr int n = SYS_N;
    constexpr int p = SYS_P;
    constexpr int q = SYS_Q;

    typedef double Scalar;
    constexpr Scalar r = 1.0;

    /* unresolved, use jordan form instead
    struct NonlinearCarClosedExpm
    {
      Eigen::Matrix<Scalar,SYS_N,SYS_N> operator()(Scalar t) const
      {
        Eigen::Matrix<Scalar,SYS_N,SYS_N> eAt;
        {4}
        return eAt;
      }
      Scalar r = Models::r;
    };
    */
    /*
    struct NonlinearCarJordanForm
    {
      typedef std::tuple<NonlinearCarSS::SystemMatrix,NonlinearCarSS::SystemMatrix> Mat;
      NonlinearCarSS::SystemMatrix J, P;
      NonlinearCarJordanForm()
      {
        {5}
      }
      Mat operator()() {
        return std::make_tuple(J,P);
      }
    };
    */

    /* compute eAt using jordan form */
    struct NonlinearCarJordanFormExpm
    {
      Eigen::Matrix<Scalar,SYS_N,SYS_N> operator()(Scalar t) const
      {
        Eigen::Matrix<Scalar,SYS_N,SYS_N> eAt, expJ;
				auto &state = x_hat;
        auto &x1_hat = state(0); auto &x2_hat = state(1); auto &x3_hat = state(2); auto &x4_hat = state(3); auto &x5_hat = state(4); 
expJ << 1, t, (1.0/2.0)*pow(t, 2), 0, 0, 0, 1, t, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, t, 0, 0, 0, 0, 1;

        eAt = P*expJ*P_inv;
        return eAt;
      }
			Eigen::Matrix<Scalar,SYS_N,SYS_N> operator()(const Eigen::Matrix<Scalar,SYS_N,1> &state, Scalar t)
      {
        Eigen::Matrix<Scalar,SYS_N,SYS_N> eAt, expJ;
        auto &x1_hat = state(0); auto &x2_hat = state(1); auto &x3_hat = state(2); auto &x4_hat = state(3); auto &x5_hat = state(4); 
expJ << 1, t, (1.0/2.0)*pow(t, 2), 0, 0, 0, 1, t, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 1, t, 0, 0, 0, 0, 1;

        P << -x3_hat*x4_hat*sin(x2_hat), cos(x2_hat), 0, -x3_hat*cos(x2_hat)/x4_hat, 0, x3_hat*x4_hat*cos(x2_hat), sin(x2_hat), 0, -x3_hat*sin(x2_hat)/x4_hat, 0, 0, x4_hat, 0, 0, 0, 0, 0, 1, 0, -x3_hat/x4_hat, 0, 0, 0, 0, 1;
P_inv = P.inverse();

        eAt = P*expJ*P_inv;
        return eAt;
      }
      void linearize(const Eigen::Matrix<Scalar,SYS_N,1> &state)
      {
        x_hat = state;
        auto &x1_hat = state(0); auto &x2_hat = state(1); auto &x3_hat = state(2); auto &x4_hat = state(3); auto &x5_hat = state(4); 
        P << -x3_hat*x4_hat*sin(x2_hat), cos(x2_hat), 0, -x3_hat*cos(x2_hat)/x4_hat, 0, x3_hat*x4_hat*cos(x2_hat), sin(x2_hat), 0, -x3_hat*sin(x2_hat)/x4_hat, 0, 0, x4_hat, 0, 0, 0, 0, 0, 1, 0, -x3_hat/x4_hat, 0, 0, 0, 0, 1;
P_inv = P.inverse();

      }
      Scalar r = Models::r;
      /* linearization state */
      Eigen::Matrix<Scalar,SYS_N,1> x_hat;
      Eigen::Matrix<Scalar,SYS_N,SYS_N> P, P_inv;
    };

    /* unresolved, use jordan form instead
    typedef StateSpace<Scalar,SYS_N,SYS_P,SYS_Q,NonlinearCarClosedExpm> NonlinearCarSS;
    typedef StateSpaceSolver<Scalar,SYS_N,SYS_P,SYS_Q,NonlinearCarSS> NonlinearCarSolver;
    */

    typedef StateSpace<Scalar,SYS_N,SYS_P,SYS_Q,NonlinearCarJordanFormExpm> NonlinearCarSS;
    typedef StateSpaceSolver<Scalar,SYS_N,SYS_P,SYS_Q,NonlinearCarSS> NonlinearCarSolver;

    NonlinearCarSS nonlinearcar;
    NonlinearCarSolver nonlinearcar_solver(nonlinearcar);

    struct NonlinearCarLinearization
    {
      typedef std::tuple<
        Eigen::Matrix<Scalar,SYS_N,SYS_N>,
        Eigen::Matrix<Scalar,SYS_N,SYS_N>,
        Eigen::Matrix<Scalar,SYS_N,SYS_N>
      > LinearizationMat;

      /* linearize matrix and create jordan form */
      LinearizationMat operator()(const Eigen::Matrix<Scalar,SYS_N,1> &state) const
      {
        Eigen::Matrix<Scalar,SYS_N,SYS_N> P, J, P_inv;
        auto &x1_hat = state(0); auto &x2_hat = state(1); auto &x3_hat = state(2); auto &x4_hat = state(3); auto &x5_hat = state(4); 
P << -x3_hat*x4_hat*sin(x2_hat), cos(x2_hat), 0, -x3_hat*cos(x2_hat)/x4_hat, 0, x3_hat*x4_hat*cos(x2_hat), sin(x2_hat), 0, -x3_hat*sin(x2_hat)/x4_hat, 0, 0, x4_hat, 0, 0, 0, 0, 0, 1, 0, -x3_hat/x4_hat, 0, 0, 0, 0, 1;
J << 0, 1, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0;
P_inv = P.inverse();

        return std::make_tuple(P, J, P_inv);
      }
      /* linearize system matrix only */
			/* TODO : resolve return type
      std::tuple<Eigen::Matrix<Scalar,SYS_N,SYS_N>> operator()(const Eigen::Matrix<Scalar,SYS_N,1> &state) const
      {

      }
			*/
      Scalar r = Models::r;
    };

    struct NonlinearCarLinearizationConstant 
    {
      void linearize(const Eigen::Matrix<Scalar,SYS_N,1> &state) 
      {
        auto &x1_hat = state(0); auto &x2_hat = state(1); auto &x3_hat = state(2); auto &x4_hat = state(3); auto &x5_hat = state(4); 
c << x2_hat*x3_hat*sin(x2_hat), -x2_hat*x3_hat*cos(x2_hat), -x3_hat*x4_hat, 0, 0;

      }
      Eigen::Matrix<Scalar,SYS_N,1>& operator()(const Eigen::Matrix<Scalar,SYS_N,1> &state)
      {
        auto &x1_hat = state(0); auto &x2_hat = state(1); auto &x3_hat = state(2); auto &x4_hat = state(3); auto &x5_hat = state(4); 
c << x2_hat*x3_hat*sin(x2_hat), -x2_hat*x3_hat*cos(x2_hat), -x3_hat*x4_hat, 0, 0;

        return c;
      }
      const Eigen::Matrix<Scalar,SYS_N,1>& operator()() const
      {
        return c;
      }
      Eigen::Matrix<Scalar,SYS_N,1> c;
    };

    // NonlinearCarCost nonlinearcar_cost;

    /* evaluate mat operation term of xbar integration */
    struct NonlinearCarIntegratorEval
    {
      NonlinearCarIntegratorEval(const NonlinearCarSS &sys, const NonlinearCarSS::StateType &c)
        : sys(sys), c(c)
      {}
      template <size_t i>
      Scalar eval(Scalar t0, Scalar t1)
      {
        NonlinearCarSS::StateType result;
        auto f = [&](Scalar t) {
          return (sys.expm(t)*c)[i];
        };
        constexpr int max_depth = 100;
        constexpr Scalar e = Scalar(1e-3);
        using integration_t = decltype(f);
        SimpsonsRuleIntegrator<Scalar,integration_t,max_depth> integrator(f,e);
        return integrator(t0,t1);
      }
      const NonlinearCarSS &sys;
      const NonlinearCarSS::StateType &c;
    };

    struct NonlinearCarGramian
    {
			void linearize(const NonlinearCarSS::StateType &state) 
			{
					this->state = state;
			}
			NonlinearCarSS::SystemMatrix operator()(const NonlinearCarSS::StateType &state, Scalar t)
      {
				linearize(state);
				auto &x1_hat = state(0); auto &x2_hat = state(1); auto &x3_hat = state(2); auto &x4_hat = state(3); auto &x5_hat = state(4); 
        NonlinearCarSS::SystemMatrix gramian;
        gramian << pow(t, 3)*((1.0/20.0)*pow(t, 2)*pow(x3_hat, 4)*pow(sin(x2_hat), 2) + (1.0/20.0)*pow(t, 2)*pow(x3_hat, 2)*pow(x4_hat, 2)*pow(sin(x2_hat), 2) - 1.0/8.0*t*x3_hat*x4_hat*sin(2*x2_hat) + (1.0/3.0)*pow(cos(x2_hat), 2))/r, pow(t, 3)*(-1.0/40.0*pow(t, 2)*pow(x3_hat, 4)*sin(2*x2_hat) - 1.0/40.0*pow(t, 2)*pow(x3_hat, 2)*pow(x4_hat, 2)*sin(2*x2_hat) + (1.0/8.0)*t*x3_hat*x4_hat*cos(2*x2_hat) + (1.0/6.0)*sin(2*x2_hat))/r, pow(t, 3)*(-1.0/8.0*t*pow(x3_hat, 3)*sin(x2_hat) - 1.0/8.0*t*x3_hat*pow(x4_hat, 2)*sin(x2_hat) + (1.0/3.0)*x4_hat*cos(x2_hat))/r, (1.0/6.0)*pow(t, 2)*(-t*x3_hat*x4_hat*sin(x2_hat) + 3*cos(x2_hat))/r, -1.0/6.0*pow(t, 3)*pow(x3_hat, 2)*sin(x2_hat)/r, pow(t, 3)*(-1.0/40.0*pow(t, 2)*pow(x3_hat, 4)*sin(2*x2_hat) - 1.0/40.0*pow(t, 2)*pow(x3_hat, 2)*pow(x4_hat, 2)*sin(2*x2_hat) + (1.0/8.0)*t*x3_hat*x4_hat*cos(2*x2_hat) + (1.0/6.0)*sin(2*x2_hat))/r, pow(t, 3)*((1.0/20.0)*pow(t, 2)*pow(x3_hat, 4)*pow(cos(x2_hat), 2) + (1.0/20.0)*pow(t, 2)*pow(x3_hat, 2)*pow(x4_hat, 2)*pow(cos(x2_hat), 2) + (1.0/8.0)*t*x3_hat*x4_hat*sin(2*x2_hat) + (1.0/3.0)*pow(sin(x2_hat), 2))/r, pow(t, 3)*((1.0/8.0)*t*pow(x3_hat, 3)*cos(x2_hat) + (1.0/8.0)*t*x3_hat*pow(x4_hat, 2)*cos(x2_hat) + (1.0/3.0)*x4_hat*sin(x2_hat))/r, (1.0/6.0)*pow(t, 2)*(t*x3_hat*x4_hat*cos(x2_hat) + 3*sin(x2_hat))/r, (1.0/6.0)*pow(t, 3)*pow(x3_hat, 2)*cos(x2_hat)/r, pow(t, 3)*(-1.0/8.0*t*pow(x3_hat, 3)*sin(x2_hat) - 1.0/8.0*t*x3_hat*pow(x4_hat, 2)*sin(x2_hat) + (1.0/3.0)*x4_hat*cos(x2_hat))/r, pow(t, 3)*((1.0/8.0)*t*pow(x3_hat, 3)*cos(x2_hat) + (1.0/8.0)*t*x3_hat*pow(x4_hat, 2)*cos(x2_hat) + (1.0/3.0)*x4_hat*sin(x2_hat))/r, (1.0/3.0)*pow(t, 3)*(pow(x3_hat, 2) + pow(x4_hat, 2))/r, (1.0/2.0)*pow(t, 2)*x4_hat/r, (1.0/2.0)*pow(t, 2)*x3_hat/r, (1.0/6.0)*pow(t, 2)*(-t*x3_hat*x4_hat*sin(x2_hat) + 3*cos(x2_hat))/r, (1.0/6.0)*pow(t, 2)*(t*x3_hat*x4_hat*cos(x2_hat) + 3*sin(x2_hat))/r, (1.0/2.0)*pow(t, 2)*x4_hat/r, t/r, 0, -1.0/6.0*pow(t, 3)*pow(x3_hat, 2)*sin(x2_hat)/r, (1.0/6.0)*pow(t, 3)*pow(x3_hat, 2)*cos(x2_hat)/r, (1.0/2.0)*pow(t, 2)*x3_hat/r, 0, t/r;
        return gramian;
      }
      NonlinearCarSS::SystemMatrix operator()(Scalar t) const
      {
				auto &x1_hat = state(0); auto &x2_hat = state(1); auto &x3_hat = state(2); auto &x4_hat = state(3); auto &x5_hat = state(4); 
        NonlinearCarSS::SystemMatrix gramian;
        gramian << pow(t, 3)*((1.0/20.0)*pow(t, 2)*pow(x3_hat, 4)*pow(sin(x2_hat), 2) + (1.0/20.0)*pow(t, 2)*pow(x3_hat, 2)*pow(x4_hat, 2)*pow(sin(x2_hat), 2) - 1.0/8.0*t*x3_hat*x4_hat*sin(2*x2_hat) + (1.0/3.0)*pow(cos(x2_hat), 2))/r, pow(t, 3)*(-1.0/40.0*pow(t, 2)*pow(x3_hat, 4)*sin(2*x2_hat) - 1.0/40.0*pow(t, 2)*pow(x3_hat, 2)*pow(x4_hat, 2)*sin(2*x2_hat) + (1.0/8.0)*t*x3_hat*x4_hat*cos(2*x2_hat) + (1.0/6.0)*sin(2*x2_hat))/r, pow(t, 3)*(-1.0/8.0*t*pow(x3_hat, 3)*sin(x2_hat) - 1.0/8.0*t*x3_hat*pow(x4_hat, 2)*sin(x2_hat) + (1.0/3.0)*x4_hat*cos(x2_hat))/r, (1.0/6.0)*pow(t, 2)*(-t*x3_hat*x4_hat*sin(x2_hat) + 3*cos(x2_hat))/r, -1.0/6.0*pow(t, 3)*pow(x3_hat, 2)*sin(x2_hat)/r, pow(t, 3)*(-1.0/40.0*pow(t, 2)*pow(x3_hat, 4)*sin(2*x2_hat) - 1.0/40.0*pow(t, 2)*pow(x3_hat, 2)*pow(x4_hat, 2)*sin(2*x2_hat) + (1.0/8.0)*t*x3_hat*x4_hat*cos(2*x2_hat) + (1.0/6.0)*sin(2*x2_hat))/r, pow(t, 3)*((1.0/20.0)*pow(t, 2)*pow(x3_hat, 4)*pow(cos(x2_hat), 2) + (1.0/20.0)*pow(t, 2)*pow(x3_hat, 2)*pow(x4_hat, 2)*pow(cos(x2_hat), 2) + (1.0/8.0)*t*x3_hat*x4_hat*sin(2*x2_hat) + (1.0/3.0)*pow(sin(x2_hat), 2))/r, pow(t, 3)*((1.0/8.0)*t*pow(x3_hat, 3)*cos(x2_hat) + (1.0/8.0)*t*x3_hat*pow(x4_hat, 2)*cos(x2_hat) + (1.0/3.0)*x4_hat*sin(x2_hat))/r, (1.0/6.0)*pow(t, 2)*(t*x3_hat*x4_hat*cos(x2_hat) + 3*sin(x2_hat))/r, (1.0/6.0)*pow(t, 3)*pow(x3_hat, 2)*cos(x2_hat)/r, pow(t, 3)*(-1.0/8.0*t*pow(x3_hat, 3)*sin(x2_hat) - 1.0/8.0*t*x3_hat*pow(x4_hat, 2)*sin(x2_hat) + (1.0/3.0)*x4_hat*cos(x2_hat))/r, pow(t, 3)*((1.0/8.0)*t*pow(x3_hat, 3)*cos(x2_hat) + (1.0/8.0)*t*x3_hat*pow(x4_hat, 2)*cos(x2_hat) + (1.0/3.0)*x4_hat*sin(x2_hat))/r, (1.0/3.0)*pow(t, 3)*(pow(x3_hat, 2) + pow(x4_hat, 2))/r, (1.0/2.0)*pow(t, 2)*x4_hat/r, (1.0/2.0)*pow(t, 2)*x3_hat/r, (1.0/6.0)*pow(t, 2)*(-t*x3_hat*x4_hat*sin(x2_hat) + 3*cos(x2_hat))/r, (1.0/6.0)*pow(t, 2)*(t*x3_hat*x4_hat*cos(x2_hat) + 3*sin(x2_hat))/r, (1.0/2.0)*pow(t, 2)*x4_hat/r, t/r, 0, -1.0/6.0*pow(t, 3)*pow(x3_hat, 2)*sin(x2_hat)/r, (1.0/6.0)*pow(t, 3)*pow(x3_hat, 2)*cos(x2_hat)/r, (1.0/2.0)*pow(t, 2)*x3_hat/r, 0, t/r;
        return gramian;
      }
			NonlinearCarSS::StateType state;
      Scalar r = Models::r;
    };

    struct NonlinearCarCost
    {
      NonlinearCarCost(const NonlinearCarSS &system, const NonlinearCarLinearizationConstant &cconstant, const NonlinearCarGramian &gramian)
        : system(system), cconstant(cconstant), g(gramian)
      {}
      Scalar operator()(const NonlinearCarSS::StateType &xi, const NonlinearCarSS::StateType &xf, Scalar t) const
      {
        Scalar cost;
        using cconst_t = decltype(cconstant());
        using d_vect_t = decltype(xf);
        const auto &sys = system;
        cconst_t c = cconstant();
        NonlinearCarIntegratorEval integrator(sys, c);
        NonlinearCarSS::StateType integration_term;
        /* TODO : optimize integration term computation */
        /* integrate each element of the integration term */
        integration_term << integrator.eval<0>(0,t),integrator.eval<1>(0,t),integrator.eval<2>(0,t),integrator.eval<3>(0,t),integrator.eval<4>(0,t);

        auto xbar = sys.expm(t)*xi + integration_term;
        auto ginv = g(t).inverse();
        cost = t+(xf-xbar).transpose()*ginv*(xf-xbar);
        return cost;
      }
      /* input weighting factor */
      Scalar r = Models::r;
      const NonlinearCarSS &system;
      const NonlinearCarLinearizationConstant &cconstant;
      const NonlinearCarGramian &g;
      /* a pointer to gramian, the implementation should set the appropriate address befor call the functor */
      // std::shared_ptr<NonlinearCarGramian> g;
    };

    /* TODO : rename to cost diff or cost derivative to avoid confusion */
    struct NonlinearCarOptTimeDiff
    {
      NonlinearCarOptTimeDiff(const NonlinearCarSS &system, const NonlinearCarLinearizationConstant &cconstant, const NonlinearCarGramian &gramian)
        : system(system), cconstant(cconstant), g(gramian)
      {}
      auto set(const NonlinearCarSS::StateType &xi, const NonlinearCarSS::StateType &xf)
      {
        this->xi = xi;
        this->xf = xf;
      }
      Scalar operator()(const Scalar t) const
      {
				using cconst_t = decltype(cconstant());
        using d_vect_t = decltype(xf);
				const auto& sys = system;
        cconst_t c = cconstant();
        NonlinearCarIntegratorEval integrator(sys, c);
        NonlinearCarSS::StateType integration_term;
        /* integrate each element of the integration term */
        integration_term << integrator.eval<0>(0,t),integrator.eval<1>(0,t),integrator.eval<2>(0,t),integrator.eval<3>(0,t),integrator.eval<4>(0,t);

        auto xbar = sys.expm(t)*xi + integration_term;
        d_vect_t d = g(t).inverse()*(xf-xbar);
        /* todo : execute around pointer instead of dereferencing */
        return (*this)(xf, d, c, g(t).inverse());
      }
			Scalar operator()(const NonlinearCarSS::StateType &xf, const NonlinearCarSS::StateType &d, const NonlinearCarSS::StateType &c, const NonlinearCarSS::SystemMatrix &ginv) const
      {
        Eigen::Matrix<Scalar,SYS_P,SYS_P> R;
        R = decltype(R)::Identity() * r;
        const auto& A = system.A;
        const auto& B = system.B;
        Scalar d_cost;
        d_cost = 1+2*(A*xf+c).transpose()*d-d.transpose()*B*R.inverse()*B.transpose()*d;
        return d_cost;
      }

      Scalar r = Models::r;
      NonlinearCarSS::StateType xi, xf;
      const NonlinearCarSS &system;
      const NonlinearCarLinearizationConstant &cconstant;
      const NonlinearCarGramian &g;
    };

    // NonlinearCarOptTimeDiff nonlinearcar_opt_time_diff;

    // NonlinearCarGramian nonlinearcar_gram;

    /* unresolved, use jordan form instead
    struct NonlinearCarCmpClosedExpm
    {
      Eigen::Matrix<Scalar,2*SYS_N,2*SYS_N> operator()(Scalar t) const
      {
        Eigen::Matrix<Scalar,2*SYS_N,2*SYS_N> eAt;        
        return eAt;
      }
      Scalar r = Models::r;
    };
    */

    /* compute eAt (of the composite system) using jordan form */
    struct NonlinearCarCmpJordanFormExpm
    {
      Eigen::Matrix<Scalar,2*SYS_N,2*SYS_N> operator()(Scalar t) const
      {
        Eigen::Matrix<Scalar,2*SYS_N,2*SYS_N> eAt, expJ;
        auto &x1_hat = state(0); auto &x2_hat = state(1); auto &x3_hat = state(2); auto &x4_hat = state(3); auto &x5_hat = state(4); 
expJ << 1, t, (1.0/2.0)*pow(t, 2), (1.0/6.0)*pow(t, 3), (1.0/24.0)*pow(t, 4), (1.0/120.0)*pow(t, 5), 0, 0, 0, 0, 0, 1, t, (1.0/2.0)*pow(t, 2), (1.0/6.0)*pow(t, 3), (1.0/24.0)*pow(t, 4), 0, 0, 0, 0, 0, 0, 1, t, (1.0/2.0)*pow(t, 2), (1.0/6.0)*pow(t, 3), 0, 0, 0, 0, 0, 0, 0, 1, t, (1.0/2.0)*pow(t, 2), 0, 0, 0, 0, 0, 0, 0, 0, 1, t, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, t, (1.0/2.0)*pow(t, 2), (1.0/6.0)*pow(t, 3), 0, 0, 0, 0, 0, 0, 0, 1, t, (1.0/2.0)*pow(t, 2), 0, 0, 0, 0, 0, 0, 0, 0, 1, t, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1;

        eAt = P*expJ*P_inv;
        return eAt;
      }
      void linearize(const Eigen::Matrix<Scalar,SYS_N,1> &state)
      {
        this->state = state;
        auto &x1_hat = state(0); auto &x2_hat = state(1); auto &x3_hat = state(2); auto &x4_hat = state(3); auto &x5_hat = state(4); 
        P << x3_hat*(pow(x3_hat, 3)*sin(x2_hat)/r + x3_hat*pow(x4_hat, 2)*sin(x2_hat)/r)*sin(x2_hat), 0, -pow(cos(x2_hat), 2)/r, 0, 0, 0, -x4_hat*cos(x2_hat)/r + (pow(x3_hat, 3)*sin(x2_hat)/r + x3_hat*pow(x4_hat, 2)*sin(x2_hat)/r)*sin(x2_hat)*cos(x2_hat)/(r*(x3_hat*x4_hat*pow(sin(x2_hat), 2)/r + x3_hat*x4_hat*pow(cos(x2_hat), 2)/r)) + (-pow(x3_hat, 3)*cos(x2_hat)/r - x3_hat*pow(x4_hat, 2)*cos(x2_hat)/r)*pow(cos(x2_hat), 2)/(r*(-x3_hat*x4_hat*pow(sin(x2_hat), 2)/r - x3_hat*x4_hat*pow(cos(x2_hat), 2)/r)), 0, 0, 0, x3_hat*(-pow(x3_hat, 3)*cos(x2_hat)/r - x3_hat*pow(x4_hat, 2)*cos(x2_hat)/r)*sin(x2_hat), -x3_hat*x4_hat*pow(sin(x2_hat), 2)/r - x3_hat*x4_hat*pow(cos(x2_hat), 2)/r, -sin(x2_hat)*cos(x2_hat)/r, 0, 0, 0, -x4_hat*sin(x2_hat)/r + (pow(x3_hat, 3)*sin(x2_hat)/r + x3_hat*pow(x4_hat, 2)*sin(x2_hat)/r)*pow(sin(x2_hat), 2)/(r*(x3_hat*x4_hat*pow(sin(x2_hat), 2)/r + x3_hat*x4_hat*pow(cos(x2_hat), 2)/r)) + (-pow(x3_hat, 3)*cos(x2_hat)/r - x3_hat*pow(x4_hat, 2)*cos(x2_hat)/r)*sin(x2_hat)*cos(x2_hat)/(r*(-x3_hat*x4_hat*pow(sin(x2_hat), 2)/r - x3_hat*x4_hat*pow(cos(x2_hat), 2)/r)), 0, 0, 0, 0, x3_hat*(-pow(x3_hat, 2)/r - pow(x4_hat, 2)/r)*sin(x2_hat), -x4_hat*cos(x2_hat)/r, 0, 0, 0, -pow(x3_hat, 2)/r - pow(x4_hat, 2)/r + x4_hat*(pow(x3_hat, 3)*sin(x2_hat)/r + x3_hat*pow(x4_hat, 2)*sin(x2_hat)/r)*sin(x2_hat)/(r*(x3_hat*x4_hat*pow(sin(x2_hat), 2)/r + x3_hat*x4_hat*pow(cos(x2_hat), 2)/r)) + x4_hat*(-pow(x3_hat, 3)*cos(x2_hat)/r - x3_hat*pow(x4_hat, 2)*cos(x2_hat)/r)*cos(x2_hat)/(r*(-x3_hat*x4_hat*pow(sin(x2_hat), 2)/r - x3_hat*x4_hat*pow(cos(x2_hat), 2)/r)), 0, 0, 0, 0, 0, -x3_hat*x4_hat*sin(x2_hat)/r, -cos(x2_hat)/r, 0, 0, -x3_hat*x4_hat*(pow(x3_hat, 3)*sin(x2_hat)/r + x3_hat*pow(x4_hat, 2)*sin(x2_hat)/r)*cos(x2_hat)/(r*(x3_hat*x4_hat*pow(sin(x2_hat), 2)/r + x3_hat*x4_hat*pow(cos(x2_hat), 2)/r)) + x3_hat*x4_hat*(-pow(x3_hat, 3)*cos(x2_hat)/r - x3_hat*pow(x4_hat, 2)*cos(x2_hat)/r)*sin(x2_hat)/(r*(-x3_hat*x4_hat*pow(sin(x2_hat), 2)/r - x3_hat*x4_hat*pow(cos(x2_hat), 2)/r)), -x4_hat/r + (pow(x3_hat, 3)*sin(x2_hat)/r + x3_hat*pow(x4_hat, 2)*sin(x2_hat)/r)*sin(x2_hat)/(r*(x3_hat*x4_hat*pow(sin(x2_hat), 2)/r + x3_hat*x4_hat*pow(cos(x2_hat), 2)/r)) + (-pow(x3_hat, 3)*cos(x2_hat)/r - x3_hat*pow(x4_hat, 2)*cos(x2_hat)/r)*cos(x2_hat)/(r*(-x3_hat*x4_hat*pow(sin(x2_hat), 2)/r - x3_hat*x4_hat*pow(cos(x2_hat), 2)/r)), 0, 0, 0, 0, -pow(x3_hat, 2)*sin(x2_hat)/r, 0, 0, 0, -pow(x3_hat, 2)*(pow(x3_hat, 3)*sin(x2_hat)/r + x3_hat*pow(x4_hat, 2)*sin(x2_hat)/r)*cos(x2_hat)/(r*(x3_hat*x4_hat*pow(sin(x2_hat), 2)/r + x3_hat*x4_hat*pow(cos(x2_hat), 2)/r)) + pow(x3_hat, 2)*(-pow(x3_hat, 3)*cos(x2_hat)/r - x3_hat*pow(x4_hat, 2)*cos(x2_hat)/r)*sin(x2_hat)/(r*(-x3_hat*x4_hat*pow(sin(x2_hat), 2)/r - x3_hat*x4_hat*pow(cos(x2_hat), 2)/r)), -x3_hat/r, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, -(-pow(x3_hat, 3)*cos(x2_hat)/r - x3_hat*pow(x4_hat, 2)*cos(x2_hat)/r)/(-x3_hat*x4_hat*pow(sin(x2_hat), 2)/r - x3_hat*x4_hat*pow(cos(x2_hat), 2)/r), 0, 0, 0, 0, 0, 0, 0, 0, 0, -(pow(x3_hat, 3)*sin(x2_hat)/r + x3_hat*pow(x4_hat, 2)*sin(x2_hat)/r)/(x3_hat*x4_hat*pow(sin(x2_hat), 2)/r + x3_hat*x4_hat*pow(cos(x2_hat), 2)/r), 0, 0, 0, 0, x3_hat*sin(x2_hat), 0, 0, 0, x3_hat*(pow(x3_hat, 3)*sin(x2_hat)/r + x3_hat*pow(x4_hat, 2)*sin(x2_hat)/r)*cos(x2_hat)/(x3_hat*x4_hat*pow(sin(x2_hat), 2)/r + x3_hat*x4_hat*pow(cos(x2_hat), 2)/r) - x3_hat*(-pow(x3_hat, 3)*cos(x2_hat)/r - x3_hat*pow(x4_hat, 2)*cos(x2_hat)/r)*sin(x2_hat)/(-x3_hat*x4_hat*pow(sin(x2_hat), 2)/r - x3_hat*x4_hat*pow(cos(x2_hat), 2)/r), 1, 0, 0, 0, -x3_hat*x4_hat*sin(x2_hat), -cos(x2_hat), 0, 0, -x3_hat*x4_hat*(pow(x3_hat, 3)*sin(x2_hat)/r + x3_hat*pow(x4_hat, 2)*sin(x2_hat)/r)*cos(x2_hat)/(x3_hat*x4_hat*pow(sin(x2_hat), 2)/r + x3_hat*x4_hat*pow(cos(x2_hat), 2)/r) + x3_hat*x4_hat*(-pow(x3_hat, 3)*cos(x2_hat)/r - x3_hat*pow(x4_hat, 2)*cos(x2_hat)/r)*sin(x2_hat)/(-x3_hat*x4_hat*pow(sin(x2_hat), 2)/r - x3_hat*x4_hat*pow(cos(x2_hat), 2)/r), -x4_hat + (pow(x3_hat, 3)*sin(x2_hat)/r + x3_hat*pow(x4_hat, 2)*sin(x2_hat)/r)*sin(x2_hat)/(x3_hat*x4_hat*pow(sin(x2_hat), 2)/r + x3_hat*x4_hat*pow(cos(x2_hat), 2)/r) + (-pow(x3_hat, 3)*cos(x2_hat)/r - x3_hat*pow(x4_hat, 2)*cos(x2_hat)/r)*cos(x2_hat)/(-x3_hat*x4_hat*pow(sin(x2_hat), 2)/r - x3_hat*x4_hat*pow(cos(x2_hat), 2)/r), 0, 0, 0, 0, -pow(x3_hat, 2)*sin(x2_hat), 0, 0, 0, -pow(x3_hat, 2)*(pow(x3_hat, 3)*sin(x2_hat)/r + x3_hat*pow(x4_hat, 2)*sin(x2_hat)/r)*cos(x2_hat)/(x3_hat*x4_hat*pow(sin(x2_hat), 2)/r + x3_hat*x4_hat*pow(cos(x2_hat), 2)/r) + pow(x3_hat, 2)*(-pow(x3_hat, 3)*cos(x2_hat)/r - x3_hat*pow(x4_hat, 2)*cos(x2_hat)/r)*sin(x2_hat)/(-x3_hat*x4_hat*pow(sin(x2_hat), 2)/r - x3_hat*x4_hat*pow(cos(x2_hat), 2)/r), -x3_hat, 0;
P_inv = P.inverse();

      }
      Scalar r = Models::r;
      /* linearization state */
      Eigen::Matrix<Scalar,SYS_N,1> state;
      Eigen::Matrix<Scalar,2*SYS_N,2*SYS_N> P, P_inv;
    };

    /*
    struct NonlinearCarCmpJordanForm
    {
      typedef std::tuple<NonlinearCarSSComposite::SystemMatrix,NonlinearCarSSComposite::SystemMatrix> Mat;
      NonlinearCarSS::SystemMatrix J, P;
      NonlinearCarCmpJordanForm()
      {
        {11}
      }
      Mat operator()() {
        return std::make_tuple(J,P);
      }
    };
    */

    /* unresolved, use jordan form instead
    typedef StateSpace<Scalar,2*SYS_N,SYS_P,SYS_Q,NonlinearCarCmpClosedExpm> NonlinearCarSSComposite;
    */
    typedef StateSpace<Scalar,2*SYS_N,SYS_P,SYS_Q,NonlinearCarCmpJordanFormExpm> NonlinearCarSSComposite;
    typedef OptimalTimeFinder<NonlinearCarOptTimeDiff> NonlinearCarOptTimeSolver;
    typedef FixedTimeLQR<NonlinearCarSS,NonlinearCarGramian> NonlinearCarFixTimeLQR;
    typedef OptTrjSolver<NonlinearCarCost,NonlinearCarOptTimeSolver,NonlinearCarFixTimeLQR,NonlinearCarSS,NonlinearCarGramian,NonlinearCarSSComposite> NonlinearCarTrajectorySolver;

    /* TODO : fix
    */

    // NonlinearCarSSComposite nonlinearcar_ss_cmp;
    // NonlinearCarFixTimeLQR nonlinearcar_ft_lqr(nonlinearcar,nonlinearcar,nonlinearcar_gram);
    // NonlinearCarOptTimeSolver nonlinearcar_opt_time_solver(nonlinearcar_opt_time_diff);
    // NonlinearCarTrajectorySolver nonlinearcar_trj_solver(nonlinearcar_cost, nonlinearcar_opt_time_solver, nonlinearcar_ft_lqr,nonlinearcar,nonlinearcar_gram,nonlinearcar_ss_cmp);

    /* TODO : fix */
    struct NonlinearCar {
      typedef NonlinearCarSS::StateType State;
      typedef NonlinearCarSS::StateType Input;

      NonlinearCar()
        /* initialize composite types */
        : opt_time_diff(state_space, cconstant, gramian)
        , cost(state_space, cconstant, gramian)
        , opt_time_solver(opt_time_diff)
        , ft_lqr(state_space, state_space, gramian)
        , solver(cost, opt_time_solver, ft_lqr, state_space, gramian, composite_ss)
      { }

      /* fundamental types, need linearization */
      NonlinearCarSS state_space;
      NonlinearCarCost cost;
      NonlinearCarGramian gramian;
      NonlinearCarSSComposite composite_ss;
      /* composite types, dependent to fundamental (or other composite) ones */
      NonlinearCarLinearizationConstant cconstant;
      NonlinearCarOptTimeDiff opt_time_diff;
      NonlinearCarOptTimeSolver opt_time_solver;
      NonlinearCarFixTimeLQR ft_lqr;
      NonlinearCarTrajectorySolver solver;

      void set_weight(Scalar r) {
        cost.r = r;
        gramian.r = r;
        opt_time_diff.r = r;
        using RMat = decltype(ft_lqr.R);
        ft_lqr.R = RMat::Identity()*r;
        state_space.exp_fn.r = r;
        composite_ss.exp_fn.r = r;
      }

      void linearize(const State &state) {
        state_space.linearize(state);
        gramian.linearize(state);
        composite_ss.linearize(state);
        cconstant.linearize(state);
      }
    };

    } // namespace model

    #endif // MODELS_HPP
    