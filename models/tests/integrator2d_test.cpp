#include <gtest/gtest.h>
#include "integrator2d.hpp"
#include "elements.hpp"
#include "collision.hpp"

typedef Models::Integrator2DOptTimeDiff TimeDiff;
typedef Models::Integrator2DOptTimeSolver TimeSolver;
typedef Models::Integrator2DTrajectorySolver TrajectorySolver;
typedef Models::Integrator2DSS Integrator2DSS;
typedef Models::Integrator2DGramian Integrator2DGramian;
typedef Models::Integrator2DClosedExpm Integrator2DClosedExpm;
typedef Models::Integrator2DSSComposite Integrator2DSSComposite;
typedef Models::Integrator2DCmpClosedExpm Integrator2DCmpClosedExpm;
typedef Models::Integrator2DSSComposite::StateType Integrator2DSSCompositeState;
typedef Models::Integrator2DSSComposite::SystemMatrix Integrator2DSSCompositeSystem;

TEST(TimeSolver, d_cost_near_zero) {
  // auto &time_diff = Models::integrator2d_opt_time_diff;
  // auto &time_solver = Models::integrator2d_opt_time_solver;

  Models::Integrator2D int2d;
  auto &time_diff = int2d.opt_time_diff;
  auto &time_solver = int2d.opt_time_solver;

  Integrator2DSS::StateType s0, s1;
  s0 << 0.0, 0.0, 1.0, 1.0;
  s1 << 0.0, 0.0, 0.0, 0.0;

  auto opt_time = time_solver.solve(s0, s1);
  time_diff.set(s0,s1);
  auto d_cost = time_diff(opt_time);
  EXPECT_NEAR(d_cost, 0.0, 0.0001) << d_cost;
}

TEST(Integrator2DClosedExpm, exp_no_inf_nan)
{
  Integrator2DClosedExpm int2d_exp;
  auto ok = true;
  std::stringstream ss;
  for(size_t i=0; i<10; i++) {
    auto t = i*0.5;
    auto m = int2d_exp(t);
    ss << "t(" << t << ") : [";
    for(size_t j=0; j<4; j++) {
      for(size_t k=0; k<4; k++)
      {
        ss << m(j,k) << (k!=3 ? " " : "; ");
        if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
      }
    }
    ss << "]" << std::endl;
  }
  EXPECT_TRUE(ok) << ss.str();
}

TEST(Integrator2DSS, exp_no_inf_nan)
{
  // Integrator2DSS int2d;
  // auto &int2d = Models::integrator2d;
  Models::Integrator2D integrator2d;
  auto &int2d = integrator2d.state_space;
  auto ok = true;
  std::stringstream ss;
  for(size_t i=0; i<10; i++) {
    auto t = i*0.5;
    auto m = int2d.expm(t);
    ss << "t(" << t << ") : [";
    for(size_t j=0; j<4; j++) {
      for(size_t k=0; k<4; k++)
      {
        ss << m(j,k) << (k!=3 ? " " : "; ");
        if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
      }
    }
    ss << "]" << std::endl;
  }
  EXPECT_TRUE(ok) << ss.str();
}

TEST(Integrator2DSSComposite, exp_no_inf_nan)
{
  // Integrator2DSSComposite ss_int2d;
  auto &ss_int2d = Models::integrator2d_ss_cmp;
  auto ok = true;
  std::stringstream ss;
  for(size_t i=0; i<10; i++) {
    auto t = i*0.5;
    auto m = ss_int2d.expm(t);
    ss << "t(" << t << ") : [";
    for(size_t j=0; j<4; j++) {
      for(size_t k=0; k<4; k++)
      {
        ss << m(j,k) << (k!=3 ? " " : "; ");
        if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
      }
    }
    ss << "]" << std::endl;
  }
  EXPECT_TRUE(ok) << ss.str();
}

TEST(Integrator2DGramian, gram_no_inf_nan)
{
  Integrator2DGramian g;
  auto ok = true;
  std::stringstream ss;
  for(size_t i=1; i<30; i++) {
    auto t = i*0.5;
    auto m = g(t);
    auto m_inv = m.inverse();
    ss << "t(" << t << ") : [";
    for(size_t j=0; j<4; j++) {
      for(size_t k=0; k<4; k++)
      {
        ss << m(j,k) << (k!=3 ? " " : "; ");
        if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
      }
    }
    ss << "]" << std::endl;
    ss << "t(" << t << ") : inverse [";
    for(size_t j=0; j<4; j++) {
      for(size_t k=0; k<4; k++)
      {
        ss << m_inv(j,k) << (k!=3 ? " " : "; ");
        if(isnan(m_inv(j,k)) || isinf(m_inv(j,k))) ok = false;
      }
    }
    ss << "]" << std::endl;
  }
  EXPECT_TRUE(ok) << ss.str();
}

TEST(Integrator2DCmpClosedExpm, exp_no_inf_nan)
{
  Integrator2DCmpClosedExpm cmp_int2d_exp;
  auto ok = true;
  std::stringstream ss;
  for(size_t i=0; i<10; i++) {
    auto t = i*0.5;
    auto m = cmp_int2d_exp(t);
    ss << "t(" << t << ") : [";
    for(size_t j=0; j<8; j++) {
      for(size_t k=0; k<8; k++)
      {
        ss << m(j,k) << (k==7 ? " " : "; ");
        if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
      }
    }
    ss << "]" << std::endl;
  }
  EXPECT_TRUE(ok) << ss.str();
}

TEST(Integrator2DSSCompositeState, composite_state_no_inf_nan)
{
  Models::Integrator2D integrator2d;
  auto &g = integrator2d.gramian;
  auto &tsolver = integrator2d.opt_time_solver;
  auto &int2d = integrator2d.state_space;
  Integrator2DSS::StateType s0, s1;
  s0 << 0.0, 0.0, 1.0, 1.0;
  s1 << 0.0, 0.0, 0.0, 0.0;
  auto opt_time = tsolver.solve(s0, s1);
  auto g_mat = g(opt_time);
  auto g_inv = g_mat.inverse();
  auto expm = int2d.expm(opt_time);
  auto d_opt = g_inv*(s1-(expm*s0));
  Integrator2DSSCompositeState cmp_state;
  cmp_state << s1, d_opt;

  auto ok = true;
  std::stringstream ss;
  ss << "("
     << s0(0) << "," << s0(1) << ","
     << s0(2) << "," << s0(3) << ")->("
     << s1(0) << "," << s1(1) << ","
     << s1(2) << "," << s1(3)
     << ")" << std::endl;
  ss << "[ ";
  ss << "opt_time(" << opt_time << ") : [";
  for(size_t k=0; k<4; k++)
  {
    ss << d_opt(k) << (k==3 ? "" : " ");
    if(isnan(d_opt(k)) || isinf(d_opt(k))) ok = false;
  }
  ss << "]" << std::endl;
  for(size_t j=0; j<8; j++) {
    ss << cmp_state(j) << (j==7 ? "" : " ");
    if(isnan(cmp_state(j)) || isinf(cmp_state(j))) ok = false;
  }
  ss << "]" << std::endl;
  EXPECT_TRUE(ok) << ss.str();
}