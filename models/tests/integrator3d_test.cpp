
#include <gtest/gtest.h>
#include "integrator3d.hpp"

typedef Models::Integrator3DOptTimeDiff Integrator3DTimeDiff;
typedef Models::Integrator3DOptTimeSolver Integrator3DTimeSolver;
typedef Models::Integrator3DTrajectorySolver Integrator3DTrajectorySolver;
typedef Models::Integrator3DSS Integrator3DSS;
typedef Models::Integrator3DGramian Integrator3DGramian;
typedef Models::Integrator3DClosedExpm Integrator3DClosedExpm;
typedef Models::Integrator3DSSComposite Integrator3DSSComposite;
typedef Models::Integrator3DCmpClosedExpm Integrator3DCmpClosedExpm;
typedef Models::Integrator3DSSComposite::StateType Integrator3DSSCompositeState;
typedef Models::Integrator3DSSComposite::SystemMatrix Integrator3DSSCompositeSystem;

TEST(Integrator3DTimeSolver, d_cost_near_zero) {
Models::Integrator3D integrator3d;
auto &time_diff = integrator3d.opt_time_diff;
auto &time_solver = integrator3d.opt_time_solver;

Integrator3DSS::StateType s0, s1;
s0 << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
s1 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;

auto opt_time = time_solver.solve(s0, s1);
time_diff.set(s0,s1);
auto d_cost = time_diff(opt_time);
EXPECT_NEAR(d_cost, 0.0, 1e-4) << d_cost;
}

TEST(Integrator3DClosedExpm, exp_no_inf_nan) {
Integrator3DClosedExpm integrator3d_exp;
auto ok = true;
std::stringstream ss;
for(size_t i=0; i<10; i++) {
auto t = i*0.5;
auto m = integrator3d_exp(t);
ss << "t(" << t << ") : [";
for(size_t j=0; j<6; j++) {
for(size_t k=0; k<6; k++) {
ss << m(j,k) << (k!=(6-1) ? " " : "; ");
if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
}
}
ss << "] " << std::endl;
}
EXPECT_TRUE(ok) << ss.str();
}

TEST(Integrator3DSS, exp_no_inf_nan) {
Models::Integrator3D integrator3d;
auto &integrator3d_ss = integrator3d.state_space;
auto ok = true;
std::stringstream ss;
for(size_t i=0; i<10; i++) {
auto t = i*0.5;
auto m = integrator3d_ss.expm(t);
ss << "t(" << t << ") : [";
for(size_t j=0; j<6; j++) {
for(size_t k=0; k<6; k++) {
ss << m(j,k) << (k!=(6-1) ? " " : "; ");
if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
}
}
ss << "]" << std::endl;
}
EXPECT_TRUE(ok) << ss.str();
}

TEST(Integrator3DSSComposite, exp_no_inf_nan) {
Models::Integrator3D integrator3d;
auto &integrator3d_ss = integrator3d.composite_ss;
auto ok = true;
std::stringstream ss;
for(size_t i=0; i<10; i++) {
auto t = i*0.5;
auto m = integrator3d_ss.expm(t);
ss << "t(" << t << ") : [";
for(size_t j=0; j<6; j++) {
for(size_t k=0; k<6; k++) {
ss << m(j,k) << (k!=(6-1) ? " " : "; ");
if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
}
}
ss << "]" << std::endl;
}
EXPECT_TRUE(ok) << ss.str();
}

TEST(Integrator3DGramian, gram_no_inf_nan) {
Integrator3DGramian g;
auto ok = true;
std::stringstream ss;
for(size_t i=1; i<30; i++) {
auto t = i*0.5;
auto m = g(t);
auto m_inv = m.inverse();
ss << "t(" << t << ") : [";
for(size_t j=0; j<6; j++) {
for(size_t k=0; k<6; k++) {
ss << m(j,k) << (k!=(6-1) ? " " : "; ");
if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
}
}
ss << "]" << std::endl;
ss << "t(" << t << ") : inverse [";
for(size_t j=0; j<6; j++) {
for(size_t k=0; k<6; k++) {
ss << m_inv(j,k) << (k!=(6-1) ? " " : "; ");
if(isnan(m_inv(j,k)) || isinf(m_inv(j,k))) ok = false;
}
}
ss << "]" << std::endl;
}
}

TEST(Integrator3DCmpClosedExpm, exp_no_inf_nan) {
Integrator3DCmpClosedExpm cmp_exp;
auto ok = true;
std::stringstream ss;
for(size_t i=0; i<10; i++) {
auto t = i*0.5;
auto m = cmp_exp(t);
ss << "t(" << t << ") : [" ;
for(size_t j=0; j<12; j++) {
for(size_t k=0; k<12; k++) {
ss << m(j,k) << (k==(12-1) ? " " : "; ");
if(isnan(m(j,k)) || isinf(m(j,k))) ok = false;
}
}
ss << "]" << std::endl;
}
EXPECT_TRUE(ok) << ss.str();
}

TEST(Integrator3DSSCompositeState, composite_state_no_inf_nan) {
Models::Integrator3D integrator3d;
auto &g = integrator3d.gramian;
auto &tsolver = integrator3d.opt_time_solver;
auto &integrator3d_ss = integrator3d.state_space;
Integrator3DSS::StateType s0, s1;
s0 << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
s1 << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
auto opt_time = tsolver.solve(s0, s1);
auto g_mat = g(opt_time);
auto g_inv = g_mat.inverse();
auto expm = integrator3d_ss.expm(opt_time);
auto d_opt = g_inv*(s1-(expm*s0));
Integrator3DSSCompositeState cmp_state;
cmp_state << s1, d_opt;
auto ok = true;
std::stringstream ss;
ss << "opt_time(" << opt_time << ") : [";
for(size_t k=0; k<6; k++) {
ss << d_opt(k) << (k==(6-1) ? "" : " ");
if(isnan(d_opt(k)) || isinf(d_opt(k))) ok = false;
}
ss << "]" << std::endl;
for(size_t j=0; j<12; j++) {
ss << cmp_state(j) << (j==(12-1) ? "" : " ");
if(isnan(cmp_state(j)) || (isinf(cmp_state(j)))) ok = false;
}
ss << "]" << std::endl;
EXPECT_TRUE(ok) << ss.str();
}
