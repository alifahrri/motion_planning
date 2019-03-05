#!/usr/bin/env python
# -*- coding: utf-8 -*-

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
  #ifndef {1}_HPP
  #define {1}_HPP

  #include "statespace.hpp"
  #include "statespacesolver.hpp"
  #include "fixedtimelqr.hpp"

  namespace Models {{

  #define SYS_N {2}
  #define SYS_P {3}
  #define SYS_Q {2}

  const int n = SYS_N;
  const int p = SYS_P;
  const int q = SYS_Q;

  typedef double Scalar;
  Scalar r = 1.0;

  struct {0}ClosedExpm
  {{
    Eigen::Matrix<Scalar,SYS_N,SYS_N> operator()(Scalar t) const
    {{
      Eigen::Matrix<Scalar,SYS_N,SYS_N> eAt;
      {4}
      return eAt;
    }}
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
  }};

  {0}Cost {1}_cost;

  struct {0}OptTimeDiff
  {{
    void set(const {0}SS::StateType &xi, const {0}SS::StateType &xf)
    {{
      {7}
    }}
    Scalar operator()(const Scalar &t) cosnt
    {{
      Scalar d_cost;
      {9}
      return d_cost;
    }}

    {6}
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
  }};

  typedef StateSpace<Scalar,2*SYS_N,SYS_P_SYS_Q> {0}SSComposite;
  typedef FixedTimeLQR<{0}SS,{0}Gramian> {0}FixTimeLQR;
  typedef OptimalTimeFinder<{0}OptTimeDiff> {0}OptTimeSolver;
  typedef OptTrjSolve<{0}Cost,{0}OptTimeSolver,{0}FixTimeLQR,{0}SS,{0}Gramian,{0}SSComposite> {0}TrajectorySolver;

  {0}SSComposeite {1}_ss_cmp;
  {0}FixTimeLQR {1}_ft_lqr({1},{1},{1}_gram);
  {0}OptTimeSolver {1}_opt_time_solver({1}_opt_time_diff);
  {0}TrajectorySolver {1}_trj_solver({1}_cost, {1}_opt_time_solver, {1}_ft_lqr,{1},{1}_gram,{1}_ss_cmp);

  struct {0} {{
    typedef {1}SS::StateType State;
    typedef {1}SS::StateType Input;

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
      ss_cmp.A << ss.a, ss.B*R.inverse()*ss.B.transpose(), {0}SS::SystemMatrix::Zero(), -ss.A.transpose();
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
    {0}OptTimeSolver ot_time_solver = {0}OptTimeSolver(opt_time_diff);
    {0}TrajectorySolver solver = {0}TrajectorySolver(cost, opt_time_solver, ft_lqr, state_space, gramian, composite_ss);

    // support for changing input weight at runtime;
    void set_weight(Scalar r) {{
      cost.r = r;
      gramian.r = r;
      opt_time_diff.r = r;
      using Rmat = decltype(ft_lqr.R);
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
    auto %ss_cmp = {1}_ss_cmp;
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

#  '// Warning : this code is generated automatically, any changes might be overwritten\n'
#  "#ifndef %s_HPP\n" % model_name.upper() +
#  "#define %s_HPP\n" % model_name.upper() +
#  "#include \"statespace.hpp\"\n"
#  "#include \"statespacesolver.hpp\"\n"
#  "//#include \"lqrsolver.hpp\"\n"
#  "#include \"fixedtimelqr.hpp\"\n"
#  "//#include \"feedbackcontroller.hpp\"\n"
#  "namespace Models {\n"
#  "#define SYS_N %s\n" % dim +
#  "#define SYS_P %s\n" % u_dim +
#  "#define SYS_Q %s\n" % dim +
#  "\n"
#  "const int n = SYS_N;\n"
#  "const int p = SYS_P;\n"
#  "const int q = SYS_Q;\n"
#  "\n"
#  "typedef double Scalar;\n"
#  '  Scalar r = 1.0;\n'
#  "\n"
#  'struct %sClosedExpm {\n' % model_name +
#  'Eigen::Matrix<Scalar,SYS_N,SYS_N> operator()(Scalar t) const\n'
#  '{\n'
#  '  Eigen::Matrix<Scalar,SYS_N,SYS_N> eAt;\n'
#  '  %s\n' % eat +
#  '  return eAt;\n'
#  '}\n'
#  '};\n'
#  "//typedef StateSpace<Scalar,SYS_N,SYS_P,SYS_Q> %sSS;\n" % model_name +
#  "typedef StateSpace<Scalar,SYS_N,SYS_P,SYS_Q,%sClosedExpm> %sSS;\n" % (model_name,model_name) +
#  "typedef StateSpaceSolver<Scalar,SYS_N,SYS_P,SYS_Q,%sSS> %sSolver;\n" % (model_name,model_name) +
#  "//typedef LQRSolver<Scalar,SYS_N,SYS_P,SYS_Q,%sSS> %sLQR;\n" % (model_name,model_name) +
#  "//typedef FeedbackController<Scalar,%sSolver,%sLQR> %sLQRControl;\n" % (model_name,model_name,model_name) +
#  'struct %sJordanForm\n' % model_name +
#  '{\n'
#  '  typedef std::tuple<%sSS::SystemMatrix,%sSS::SystemMatrix> Mat;\n' %(model_name,model_name) +
#  '  %sSS::SystemMatrix J, P;\n' % model_name +
#  '  %sJordanForm()\n' % model_name +
#  '  {\n'
#  '    %s\n' % jordan +
#  '  }\n'
#  '  Mat operator()(){\n'
#  '    return std::make_tuple(J,P);\n'
#  '  }\n'
#  '};\n'
#  '%sSS %s;\n' %(model_name, model_name.lower()) +
#  '%sSolver %s_solver(%s);\n' %(model_name, model_name.lower(), model_name.lower()) +
#  '\n'
#  '//%sLQR %s_lqr(%s);\n' %(model_name, model_name.lower(), model_name.lower()) +
#  '//%sLQRControl %s_lqr_control(%s_solver, %s_lqr);\n' %(model_name, model_name.lower(), model_name.lower(), model_name.lower()) +
#  'struct %sCost\n' % model_name +
#  '{\n'
#  '  Scalar operator()(const %sSS::StateType &xi, const %sSS::StateType &xf, const Scalar &t) const\n' %(model_name,model_name) +
#  '  {\n'
#  '    Scalar cost;\n'
#  '    %s\n' % if_var +
#  '    %s\n' % if_set +
#  '    %s\n' % c +
#  '    return cost;\n'
#  '  }\n'
#  '} %s_cost;\n' % model_name.lower() +
#  'struct %sOptTimeDiff\n' % model_name +
#  '{\n'
#  '  void set(const %sSS::StateType &xi, const %sSS::StateType &xf)\n' %(model_name,model_name) +
#  '  {\n'
#  '    %s\n' % if_set +
#  '  }\n'
#  '  Scalar operator()(const Scalar &t) const\n'
#  '  {\n'
#  '    Scalar d_cost;\n'
#  '    %s\n' % dc +
#  '    return d_cost;\n'
#  '  }\n'
#  '  %s\n' %if_var +
#  '} %s_opt_time_diff;\n' % model_name.lower() +
#  'struct %sGramian {\n' %model_name +
#  '  %sSS::SystemMatrix operator()(Scalar t) const\n' %model_name +
#  '  {\n'
#  '    %sSS::SystemMatrix G;\n' %model_name +
#  '    %s\n' % g +
#  '    return G;\n'
#  '  }\n'
#  '} %s_gram;\n' %model_name.lower() +
#  'struct %sCmpClosedExpm {\n' % model_name +
#  'Eigen::Matrix<Scalar,2*SYS_N,2*SYS_N> operator()(Scalar t) const\n'
#  '{\n'
#  '  Eigen::Matrix<Scalar,2*SYS_N,2*SYS_N> eAt;\n'
#  '  %s\n' % cmp_exp +
#  '  return eAt;\n'
#  '}\n'
#  '};\n'
#  '//typedef StateSpace<Scalar,2*SYS_N,SYS_P,SYS_Q> %sSSComposite;\n' % model_name +
#  'typedef StateSpace<Scalar,2*SYS_N,SYS_P,SYS_Q,%sCmpClosedExpm> %sSSComposite;\n' % (model_name,model_name) +
#  'typedef FixedTimeLQR<%sSS,%sGramian> %sFixTimeLQR;\n' % (model_name,model_name,model_name) +
#  'typedef OptimalTimeFinder<%sOptTimeDiff> %sOptTimeSolver;\n' % (model_name,model_name) +
#  'typedef OptTrjSolver<%sCost,%sOptTimeSolver,%sFixTimeLQR,%sSS,%sGramian,%sSSComposite> %sTrajectorySolver;\n' \
#  %(model_name,model_name,model_name,model_name,model_name,model_name,model_name) +
#  '%sSSComposite %s_ss_cmp;\n' % (model_name, model_name.lower()) +
#  '%sFixTimeLQR %s_ft_lqr(%s, %s, %s_gram);\n' \
#  % (model_name, model_name.lower(), model_name.lower(), model_name.lower(), model_name.lower()) +
#  '%sOptTimeSolver %s_opt_time_solver(%s_opt_time_diff);\n' \
#  % (model_name, model_name.lower(), model_name.lower()) +
#  '%sTrajectorySolver %s_trj_solver(%s_cost, %s_opt_time_solver,%s_ft_lqr,%s,%s_gram,%s_ss_cmp);\n' \
#  % (model_name, model_name.lower(), model_name.lower(), model_name.lower(), model_name.lower(), model_name.lower(), model_name.lower(), model_name.lower()) +
#  '\n'
#  'struct %s {\n' % model_name +
#  '  typedef %sSS::StateType State;\n' % model_name +
#  '  typedef %sSS::InputType Input;\n' % model_name +
#  '  \n'
#  '  %s()\n' % model_name +
#  '  {\n'
#  '    auto &ss = this->state_space;\n'
#  '    ss.%s\n' % a_str +
#  '    ss.%s\n' % b_str +
#  '    ss.%s\n' % c_str +
#  '    // not used anymore, use closed form of exp mat instead\n'
#  '    // TODO : remove\n'
#  '    %sJordanForm %s_jordan_form;\n' % (model_name, model_name.lower()) +
#  '    auto t = %s_jordan_form();\n' % model_name.lower() +
#  '    ss.D = std::get<0>(t);\n'
#  '    ss.P = std::get<1>(t);\n'
#  '    ss.P_inv = ss.P.inverse();\n'
#  '    \n'
#  '    auto R = this->ft_lqr.R;\n'
#  '    auto &ss_cmp = this->composite_ss;\n'
#  '    ss_cmp.A << ss.a, ss.B*R.inverse()*ss.B.transpose(), %sSS::SystemMatrix::Zero(), -ss.A.transpose();\n' \
#  % model_name +
#  '    ss_cmp.%s\n' % cmp_P_str +
#  '    ss_cmp.%s\n' % cmp_J_str +
#  '    ss_cmp.P_inv = ss_cmp.P.inverse();\n'
#  '  }\n'
#  '  %sSS state_space;\n' % model_name +
#  '  %sCost cost;\n' % model_name +
#  '  %sGramian gramian;\n' % model_name +
#  '  %sSSComposite composite_ss;\n' % model_name +
#  '  %sOptTimeDiff opt_time_diff;\n' % model_name +
#  '  %sFixTimeLQR ft_lqr = %sFixTimeLQR(state_space, state_space, gramian);\n' % (model_name, model_name) +
#  '  %sOptTimeSolver opt_time_solver = %sOptTimeSolver(opt_time_diff);\n' % model_name +
#  '  %sTrajectorySolver solver = %sTrajectorySolver(cost, opt_time_solver, ft_lqr, state_space, gramian, composite_ss);\n' % (model_name, model_name) +
#  '  \n'
#  '  // support for changing input weight at runtime:\n'
#  '  void set_weight(Scalar r) {\n'
#  '    cost.r = r;\n'
#  '    gramian.r = r;\n'
#  '    opt_time_diff.r = r;\n'
#  '    using RMat = decltype(ft_lqr.R);\n'
#  '    ft_lqr.R = RMat::Identity()*r;\n'
#  '    state_space.exp_fn.r = r;\n'
#  '    composite_ss.exp_fn.r = r;\n'
#  '  }\n'
#  '};\n'
#  'void init_%s()\n' % model_name.lower() +
#  '{\n'
#  '  auto &ss = %s;\n' % model_name.lower() +
#  '  %sJordanForm %s_jordan_form;\n' %(model_name, model_name.lower()) +
#  '  ss.%s\n' % a_str +
#  '  ss.%s\n' % b_str +
#  '  ss.%s\n' % c_str +
#  '  auto t = %s_jordan_form();\n' % model_name.lower() +
#  '  ss.D = std::get<0>(t);\n'
#  '  ss.P = std::get<1>(t);\n'
#  '  ss.P_inv = ss.P.inverse();\n'
#  '\n'
#  # '  Scalar r = 1.0;\n'
#  '  auto R = %s_ft_lqr.R;\n' % model_name.lower() +
#  '  auto &ss_cmp = %s_ss_cmp;\n' % model_name.lower() +
#  '  ss_cmp.A << ss.A,             ss.B*R.inverse()*ss.B.transpose(),\n'
#  '              %sSS::SystemMatrix::Zero(), -ss.A.transpose();\n' % model_name +
#  '  ss_cmp.%s\n' % cmp_P_str +
#  '  ss_cmp.%s\n' % cmp_J_str +
#  '  ss_cmp.P_inv = ss_cmp.P.inverse();\n'
#  '\n'
#  '/*'
#  '  std::cout << "test expm :" << std::endl\n'
#  '            << "A.expm(0.0) :"<< std::endl << ss.expm(0.0) << std::endl\n'
#  '            << "A.expm(1.0) :"<< std::endl << ss.expm(1.0) << std::endl\n'
#  '            << "A.expm(-1.0) :"<< std::endl << ss.expm(-1.0) << std::endl\n'
#  '            << "A.expm(2.5) :"<< std::endl << ss.expm(2.5) << std::endl\n'
#  '            << "A.expm(-2.5) :"<< std::endl << ss.expm(-2.5) << std::endl;\n'
#  '\n'
#  '  std::cout << "test composite matrix" << std::endl\n'
#  '            << "ss_cmp.A :"<< std::endl << ss_cmp.A << std::endl\n'
#  '            << "ss_cmp.expm(0.0) :"<< std::endl << ss_cmp.expm(0.0) << std::endl\n'
#  '            << "ss_cmp.expm(-1.0) :"<< std::endl << ss_cmp.expm(-1.0) << std::endl\n'
#  '            << "ss_cmp.expm(1.0) :"<< std::endl << ss_cmp.expm(1.0) << std::endl;\n'
#  '*/\n'
#  '}\n'
#  '\n'
#  '%sSS& get_%s()\n' % (model_name, model_name.lower()) +
#  '{\n'
#  'return %s;\n' %model_name.lower() +
#  '}\n'
#  '\n'
#  '}\n'
#  '\n'
#  '#endif // MODELS_HPP\n'
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

  typedef Models::{0}OptTimeDiff TimeDiff;
  typedef Models::{0}OptTimeSolver TimeSolver;
  typedef Models::{0}TrajectorySolver TrajectorySolver;
  typedef Models::{0}SS {0}SS;
  typedef Models::{0}Gramian {0}Gramian;
  typedef Models::{0}ClosedExpm {0}ClosedExpm;
  typedef Models::{0}SSComposite {0}SSComposite;
  typedef Models::{0}CmpClosedExpm {0}CmpClosedExpm;
  typedef Models::{0}SSComposite::StateType {0}SSCompositeState;
  typedef Models::{0}SSComposite::SystemMatrix {0}SSCompositeSystem;

  TEST(TimeSolver, d_cost_near_zero) {{
    Models::{0} {1};
    auto &time_diff = {1}.opt_time_diff;
    auto &time_solver = {1}.opt_time_solver;

    {1}SS::StateType s0, s1;
    s0 << {2};
    s1 << {3};

    auto opt_time = time_solver.solve(s0, s1);
    time_diff.set(s0,s1);
    auto d_cost = time_diff(opt_time);
    EXPECT_NEAR(d_cost, 0.0, 1e-4) << d_cost;
  }}
  """.format(
    model_name,
    model_name.lower(),
    ['1.0, ' if i==(dim-1) else '1.0;' for i in range(dim)],
    ['0.0, ' if i==(dim-1) else '0.0;' for i in range(dim)],
  )
  )
  return test_code
