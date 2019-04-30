
    #include <gtest/gtest.h>
    #include "nonlinearcar.hpp"

    typedef Models::NonlinearCarOptTimeDiff NonlinearCarTimeDiff;
    // typedef Models::NonlinearCarOptTimeSolver NonlinearCarTimeSolver;
    // typedef Models::NonlinearCarTrajectorySolver NonlinearCarTrajectorySolver;
    typedef Models::NonlinearCarSS NonlinearCarSS;
    typedef Models::NonlinearCarGramian NonlinearCarGramian;
    typedef Models::NonlinearCarJordanFormExpm NonlinearCarClosedExpm;
    typedef Models::NonlinearCarSSComposite NonlinearCarSSComposite;
    typedef Models::NonlinearCarCmpJordanFormExpm NonlinearCarCmpClosedExpm;
    typedef Models::NonlinearCarSSComposite::StateType NonlinearCarSSCompositeState;
    typedef Models::NonlinearCarSSComposite::SystemMatrix NonlinearCarSSCompositeSystem;

    TEST(NonlinearCarCost, cost_no_inf_nan) {
      Models::NonlinearCar nonlinearcar;
      auto &solver = nonlinearcar.solver;
      /* initial state */
      Eigen::Matrix<double,5,1> init_state; 
init_state << 0,0,0,0,0;
      /* final state */
      Eigen::Matrix<double,5,1> state; 
state << 1,1,1,1,1;
      /* linearization */
      nonlinearcar.linearize(state);
      auto cost = solver.cost(init_state,state);
      auto ok = true;
      if(isinf(std::get<0>(cost)) || isnan(std::get<0>(cost)))
        ok = false;
      if(isinf(std::get<1>(cost)) || isnan(std::get<1>(cost)))
        ok = false;
      EXPECT_TRUE(ok);
    }

    TEST(NonlinearCarTrajectorySolver, trajectory_no_inf_nan) {
      Models::NonlinearCar nonlinearcar;
      auto &solver = nonlinearcar.solver;
      /* initial state */
      Eigen::Matrix<double,5,1> init_state; 
init_state << 0,0,0,0,0;
      /* final state */
      Eigen::Matrix<double,5,1> state; 
state << 1,1,1,1,1;
      /* linearization */
      nonlinearcar.linearize(state);
      auto trajectory = solver.solve(init_state,state);
      auto ok = true;
      for(const auto &t : trajectory) {
        const auto time = std::get<0>(t);
        const auto &state = std::get<1>(t);
        const auto &control = std::get<2>(t);
        if(isinf(time) || isnan(time))
          ok = false;
        for(size_t i=0; i<5; i++)
          if(isinf(state(i)) || isnan(state(i)))
            ok = false;
        /* TODO : check input */
      }
      EXPECT_TRUE(ok);
    }

    TEST(NonlinearCarTimeSolver, d_cost_near_zero) {
      Models::NonlinearCar nonlinearcar;
      auto &time_diff = nonlinearcar.opt_time_diff;
      auto &time_solver = nonlinearcar.opt_time_solver;

      /* initial state */
      Eigen::Matrix<double,5,1> init_state; 
init_state << 0,0,0,0,0;
      /* final state */
      Eigen::Matrix<double,5,1> state; 
state << 1,1,1,1,1;
      /* linearization */
      nonlinearcar.linearize(state);

      auto opt_time = time_solver.solve(init_state,state);
      time_diff.set(init_state,state);
      auto d_cost = time_diff(opt_time);
      EXPECT_NEAR(d_cost, 0.0, 1e-4) << d_cost;
    }

    TEST(NonlinearCarGramian, gram_no_inf_nan) {
      NonlinearCarGramian g;
      auto ok = true;
      std::stringstream ss;
      /* linearization */
      Eigen::Matrix<double,5,1> state; 
state << 1,1,1,1,1;
      g.linearize(state);
      for(size_t i=1; i<30; i++) {
        auto t = i*0.5;
        auto m = g(t);
        auto m_inv = m.inverse();
        ss << "t(" << t << ") : [";
        for(size_t j=0; j<5; j++) {
          for(size_t k=0; k<5; k++) {
            ss << m(j,k) << (k!=(5-1) ? " " : "; ");
            if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
          }
        }
        ss << "]" << std::endl;
        ss << "t(" << t << ") : inverse [";
        for(size_t j=0; j<5; j++) {
          for(size_t k=0; k<5; k++) {
            ss << m_inv(j,k) << (k!=(5-1) ? " " : "; ");
            if(isnan(m_inv(j,k)) || isinf(m_inv(j,k))) ok = false;
          }
        }
        ss << "]" << std::endl;
      }
    }

    TEST(NonlinearCarClosedExpm, exp_no_inf_nan) {
			NonlinearCarClosedExpm nonlinearcar_exp;
			auto ok = true;
			std::stringstream ss;
      /* linearization */
      Eigen::Matrix<double,5,1> state; 
state << 1,1,1,1,1;
      nonlinearcar_exp.linearize(state);
			for(size_t i=0; i<10; i++) {
				auto t = i*0.5;
				auto m = nonlinearcar_exp(t);
				ss << "t(" << t << ") : [";
				for(size_t j=0; j<5; j++) {
					for(size_t k=0; k<5; k++) {
						ss << m(j,k) << (k!=(5-1) ? " " : "; ");
						if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
					}
				}
				ss << "] " << std::endl;
			}
			EXPECT_TRUE(ok) << ss.str();
	  }

    TEST(NonlinearCarSS, exp_no_inf_nan) {
      Models::NonlinearCar nonlinearcar;
      auto &nonlinearcar_ss = nonlinearcar.state_space;
      auto ok = true;
      std::stringstream ss;
      /* linearization */
      Eigen::Matrix<double,5,1> state; 
state << 1,1,1,1,1;
      nonlinearcar_ss.linearize(state);
      for(size_t i=0; i<10; i++) {
        auto t = i*0.5;
        auto m = nonlinearcar_ss.expm(t);
        ss << "t(" << t << ") : [";
        for(size_t j=0; j<5; j++) {
          for(size_t k=0; k<5; k++) {
            ss << m(j,k) << (k!=(5-1) ? " " : "; ");
            if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
          }
        }
        ss << "]" << std::endl;
      }
      EXPECT_TRUE(ok) << ss.str();
    }

    TEST(NonlinearCarCmpClosedExpm, exp_no_inf_nan) {
			NonlinearCarCmpClosedExpm nonlinearcar_exp;
			auto ok = true;
			std::stringstream ss;
      /* linearization */
      Eigen::Matrix<double,5,1> state; 
state << 1,1,1,1,1;
      nonlinearcar_exp.linearize(state);
			for(size_t i=0; i<10; i++) {
				auto t = i*0.5;
				auto m = nonlinearcar_exp(t);
				ss << "t(" << t << ") : [";
				for(size_t j=0; j<10; j++) {
					for(size_t k=0; k<10; k++) {
						ss << m(j,k) << (k!=(5-1) ? " " : "; ");
						if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
					}
				}
				ss << "] " << std::endl;
			}
			EXPECT_TRUE(ok) << ss.str();
	  }

    TEST(NonlinearCarSSComposite, exp_no_inf_nan) {
      Models::NonlinearCar nonlinearcar;
      auto &nonlinearcar_ss = nonlinearcar.composite_ss;
      auto ok = true;
      std::stringstream ss;
      /* linearization */
      Eigen::Matrix<double,5,1> state; 
state << 1,1,1,1,1;
      nonlinearcar_ss.linearize(state);
      for(size_t i=0; i<10; i++) {
        auto t = i*0.5;
        auto m = nonlinearcar_ss.expm(t);
        ss << "t(" << t << ") : [";
        for(size_t j=0; j<10; j++) {
          for(size_t k=0; k<10; k++) {
            ss << m(j,k) << (k!=(10-1) ? " " : "; ");
            if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
          }
        }
        ss << "]" << std::endl;
      }
      EXPECT_TRUE(ok) << ss.str();
    }
    