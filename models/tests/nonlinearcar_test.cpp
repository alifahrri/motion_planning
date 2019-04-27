
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
    