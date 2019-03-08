
// Warning : this code is generated automatically, any changes might be overwritten
#ifndef Integrator3D_HPP
#define Integrator3D_HPP

#include "statespace.hpp"
#include "statespacesolver.hpp"
#include "fixedtimelqr.hpp"

namespace Models {

#define SYS_N 6
#define SYS_P 3
#define SYS_Q 6

constexpr int n = SYS_N;
constexpr int p = SYS_P;
constexpr int q = SYS_Q;

typedef double Scalar;
Scalar r = 1.0;

struct Integrator3DClosedExpm
{
Eigen::Matrix<Scalar,SYS_N,SYS_N> operator()(Scalar t) const
{
Eigen::Matrix<Scalar,SYS_N,SYS_N> eAt;
eAt << 1, 0, 0, t, 0, 0, 0, 1, 0, 0, t, 0, 0, 0, 1, 0, 0, t, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
return eAt;
}
Scalar r = Models::r;
};

typedef StateSpace<Scalar,SYS_N,SYS_P,SYS_Q,Integrator3DClosedExpm> Integrator3DSS;
typedef StateSpaceSolver<Scalar,SYS_N,SYS_P,SYS_Q,Integrator3DSS> Integrator3DSolver;
struct Integrator3DJordanForm
{
typedef std::tuple<Integrator3DSS::SystemMatrix,Integrator3DSS::SystemMatrix> Mat;
Integrator3DSS::SystemMatrix J, P;
Integrator3DJordanForm()
{
P << 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 1;
J << 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0;
}
Mat operator()() {
return std::make_tuple(J,P);
}
};

Integrator3DSS integrator3d;
Integrator3DSolver integrator3d_solver(integrator3d);

struct Integrator3DCost
{
Scalar operator()(const Integrator3DSS::StateType &xi, const Integrator3DSS::StateType &xf, const Scalar &t) const
{
Scalar cost;
Scalar x0i, x0f, x1i, x1f, x2i, x2f, x3i, x3f, x4i, x4f, x5i, x5f; 
x0i = xi(0); x0f = xf(0); x1i = xi(1); x1f = xf(1); x2i = xi(2); x2f = xf(2); x3i = xi(3); x3f = xf(3); x4i = xi(4); x4f = xf(4); x5i = xi(5); x5f = xf(5); 
cost = t + (x3f - x3i)*(4*r*(x3f - x3i)/t - 6*r*(-t*x3i + x0f - x0i)/pow(t, 2)) + (x4f - x4i)*(4*r*(x4f - x4i)/t - 6*r*(-t*x4i + x1f - x1i)/pow(t, 2)) + (x5f - x5i)*(4*r*(x5f - x5i)/t - 6*r*(-t*x5i + x2f - x2i)/pow(t, 2)) + (-6*r*(x3f - x3i)/pow(t, 2) + 12*r*(-t*x3i + x0f - x0i)/pow(t, 3))*(-t*x3i + x0f - x0i) + (-6*r*(x4f - x4i)/pow(t, 2) + 12*r*(-t*x4i + x1f - x1i)/pow(t, 3))*(-t*x4i + x1f - x1i) + (-6*r*(x5f - x5i)/pow(t, 2) + 12*r*(-t*x5i + x2f - x2i)/pow(t, 3))*(-t*x5i + x2f - x2i);
return cost;
}
Scalar r = Models::r;
};

Integrator3DCost integrator3d_cost;

struct Integrator3DOptTimeDiff
{
void set(const Integrator3DSS::StateType &xi, const Integrator3DSS::StateType &xf)
{
x0i = xi(0); x0f = xf(0); x1i = xi(1); x1f = xf(1); x2i = xi(2); x2f = xf(2); x3i = xi(3); x3f = xf(3); x4i = xi(4); x4f = xf(4); x5i = xi(5); x5f = xf(5); 
}
Scalar operator()(const Scalar &t) const
{
Scalar d_cost;
d_cost = -2*x3f*(-6*r*(x3f - x3i)/pow(t, 2) + 12*r*(-t*x3i + x0f - x0i)/pow(t, 3)) - 2*x4f*(-6*r*(x4f - x4i)/pow(t, 2) + 12*r*(-t*x4i + x1f - x1i)/pow(t, 3)) - 2*x5f*(-6*r*(x5f - x5i)/pow(t, 2) + 12*r*(-t*x5i + x2f - x2i)/pow(t, 3)) + 1 - pow(4*r*(x3f - x3i)/t - 6*r*(-t*x3i + x0f - x0i)/pow(t, 2), 2)/r - pow(4*r*(x4f - x4i)/t - 6*r*(-t*x4i + x1f - x1i)/pow(t, 2), 2)/r - pow(4*r*(x5f - x5i)/t - 6*r*(-t*x5i + x2f - x2i)/pow(t, 2), 2)/r;
return d_cost;
}

Scalar x0i, x0f, x1i, x1f, x2i, x2f, x3i, x3f, x4i, x4f, x5i, x5f; 
Scalar r = Models::r;
};

Integrator3DOptTimeDiff integrator3d_opt_time_diff;

struct Integrator3DGramian
{
Integrator3DSS::SystemMatrix operator()(Scalar t) const
{
Integrator3DSS::SystemMatrix G;
G << (1.0/3.0)*pow(t, 3)/r, 0, 0, (1.0/2.0)*pow(t, 2)/r, 0, 0, 0, (1.0/3.0)*pow(t, 3)/r, 0, 0, (1.0/2.0)*pow(t, 2)/r, 0, 0, 0, (1.0/3.0)*pow(t, 3)/r, 0, 0, (1.0/2.0)*pow(t, 2)/r, (1.0/2.0)*pow(t, 2)/r, 0, 0, t/r, 0, 0, 0, (1.0/2.0)*pow(t, 2)/r, 0, 0, t/r, 0, 0, 0, (1.0/2.0)*pow(t, 2)/r, 0, 0, t/r;
return G;
}
Scalar r = Models::r;
};

Integrator3DGramian integrator3d_gram;

struct Integrator3DCmpClosedExpm
{
Eigen::Matrix<Scalar,2*SYS_N,2*SYS_N> operator()(Scalar t) const
{
Eigen::Matrix<Scalar,2*SYS_N,2*SYS_N> eAt;
eAt << 1, 0, 0, t, 0, 0, -1.0/6.0*pow(t, 3)/r, 0, 0, (1.0/2.0)*pow(t, 2)/r, 0, 0, 0, 1, 0, 0, t, 0, 0, -1.0/6.0*pow(t, 3)/r, 0, 0, (1.0/2.0)*pow(t, 2)/r, 0, 0, 0, 1, 0, 0, t, 0, 0, -1.0/6.0*pow(t, 3)/r, 0, 0, (1.0/2.0)*pow(t, 2)/r, 0, 0, 0, 1, 0, 0, -1.0/2.0*pow(t, 2)/r, 0, 0, t/r, 0, 0, 0, 0, 0, 0, 1, 0, 0, -1.0/2.0*pow(t, 2)/r, 0, 0, t/r, 0, 0, 0, 0, 0, 0, 1, 0, 0, -1.0/2.0*pow(t, 2)/r, 0, 0, t/r, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, -t, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, -t, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, -t, 0, 0, 1;
return eAt;
}
Scalar r = Models::r;
};

typedef StateSpace<Scalar,2*SYS_N,SYS_P,SYS_Q,Integrator3DCmpClosedExpm> Integrator3DSSComposite;
typedef FixedTimeLQR<Integrator3DSS,Integrator3DGramian> Integrator3DFixTimeLQR;
typedef OptimalTimeFinder<Integrator3DOptTimeDiff> Integrator3DOptTimeSolver;
typedef OptTrjSolver<Integrator3DCost,Integrator3DOptTimeSolver,Integrator3DFixTimeLQR,Integrator3DSS,Integrator3DGramian,Integrator3DSSComposite> Integrator3DTrajectorySolver;

Integrator3DSSComposite integrator3d_ss_cmp;
Integrator3DFixTimeLQR integrator3d_ft_lqr(integrator3d,integrator3d,integrator3d_gram);
Integrator3DOptTimeSolver integrator3d_opt_time_solver(integrator3d_opt_time_diff);
Integrator3DTrajectorySolver integrator3d_trj_solver(integrator3d_cost, integrator3d_opt_time_solver, integrator3d_ft_lqr,integrator3d,integrator3d_gram,integrator3d_ss_cmp);

struct Integrator3D {
typedef Integrator3DSS::StateType State;
typedef Integrator3DSS::StateType Input;

Integrator3D()
{
auto &ss = this->state_space;
ss.A << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
ss.B << 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
ss.C << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
// jordan form should be depecrated, use closed form of exp mat instead
// TODO : remove
Integrator3DJordanForm integrator3d_jordan_form;
auto t = integrator3d_jordan_form();
ss.D = std::get<0>(t);
ss.P = std::get<1>(t);
ss.P_inv = ss.P.inverse();

auto R = this->ft_lqr.R;
auto &ss_cmp = this->composite_ss;
ss_cmp.A << ss.A, ss.B*R.inverse()*ss.B.transpose(), Integrator3DSS::SystemMatrix::Zero(), -ss.A.transpose();
ss_cmp.P << -1/r, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1/r, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1/r, 0, 0, 0, 0, -1/r, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1/r, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1/r, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0;
ss_cmp.D << 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
ss_cmp.P_inv = ss_cmp.P.inverse();
}

Integrator3DSS state_space;
Integrator3DCost cost;
Integrator3DGramian gramian;
Integrator3DSSComposite composite_ss;
Integrator3DOptTimeDiff opt_time_diff;
Integrator3DFixTimeLQR ft_lqr = Integrator3DFixTimeLQR(state_space, state_space, gramian);
Integrator3DOptTimeSolver opt_time_solver = Integrator3DOptTimeSolver(opt_time_diff);
Integrator3DTrajectorySolver solver = Integrator3DTrajectorySolver(cost, opt_time_solver, ft_lqr, state_space, gramian, composite_ss);

// support for changing input weight at runtime;
void set_weight(Scalar r) {
cost.r = r;
gramian.r = r;
opt_time_diff.r = r;
using RMat = decltype(ft_lqr.R);
ft_lqr.R = RMat::Identity()*r;
state_space.exp_fn.r = r;
composite_ss.exp_fn.r = r;
}
};

// [[deprecated(use Integrator3D class instead)]]
void init_integrator3d()
{
auto &ss = integrator3d;
Integrator3DJordanForm integrator3d_jordan_form;
ss.A << 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
ss.B << 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 1, 0, 0, 0, 1;
ss.C << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 1;
auto t = integrator3d_jordan_form();
ss.D = std::get<0>(t);
ss.P = std::get<1>(t);
ss.P_inv = ss.P.inverse();

auto R = integrator3d_ft_lqr.R;
auto &ss_cmp = integrator3d_ss_cmp;
ss_cmp.A << ss.A, ss.B*R.inverse()*ss.B.transpose(), Integrator3DSS::SystemMatrix::Zero(), -ss.A.transpose();
ss_cmp.P << -1/r, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1/r, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1/r, 0, 0, 0, 0, -1/r, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1/r, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1/r, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -1, 0;
ss_cmp.D << 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
ss_cmp.P_inv = ss_cmp.P.inverse();
}

// [[deprecated(use Integrator3D class instead)]]
Integrator3DSS get_integrator3d()
{
return integrator3d;
}

} // namespace model

#endif // MODELS_HPP
