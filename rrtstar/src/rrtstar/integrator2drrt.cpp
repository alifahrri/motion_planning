#include "integrator2drrt.hpp"

using namespace Kinodynamic;

NeighborRadius::NeighborRadius() { s = std::pow(2,4)*(1+1/4)*SAMPLE_VOLUME; }

double NeighborRadius::operator()(const TreeInt2D::Index &i) {
  return s*scale*std::log(i+1)/(i+1);
}

bool CollisionChecker::operator()(const state_t &s0, const state_t &s1) const
{
#ifdef NO_OBS
  return false;
#else
  bool collision = false;

  if(env.collide(s1))
    collision = true;

  if(env.collide(s0))
    collision = true;

  if(!collision) {
    auto trajectory = solver.solve<segment>(s0,s1);
    for(size_t i=1; i<trajectory.size(); i++) {
      const auto &t0 = std::get<1>(trajectory[i-1]);
      const auto &t1 = std::get<1>(trajectory[i]);
      if(env.collide(t0,t1)) {
        collision = true;
        break;
      }
    }
  }
  return collision;
#endif
}

CollisionChecker::CollisionChecker(Models::Integrator2DTrajectorySolver &solver, Robosoccer<scalar, obs_count> &env)
  : solver(solver), env(env)
{
  rg = new RandomGen<2,scalar>({-SAMPLE_X0,-SAMPLE_X1},{SAMPLE_X0,SAMPLE_X1});
}

void CollisionChecker::dump_text(const string &env_file, const string &collision_file) {
  Logger logger;
  for(const auto &o : env.obs)
    logger << std::get<0>(o) << " " << std::get<1>(o) << '\n';
  logger.save(env_file);

  /*
    logger.clear();
    for(const auto& c : collisions)
      logger << "(" << std::get<0>(c) << "," << std::get<1>(c) << ")\n";
    logger.save(collision_file);
    */
}

void CollisionChecker::from_text(const string &file) {
  std::string str;
  std::ifstream stream(file);
  int i=0;
  while(std::getline(stream, str)) {
    std::vector<std::string> vstr;
    std::istringstream istr(str);
    std::string sstr;
    while(std::getline(istr, sstr, ' '))
      vstr.push_back(sstr);
    env.obs.at(i) = std::make_tuple(std::stold(vstr.at(0)),std::stold(vstr.at(1)));
    i++;
  }
  stream.close();
}

void CollisionChecker::setRandomObstacles() {
  for(auto& o : env.obs) {
    std::get<0>(o) = (*rg)(0);
    std::get<1>(o) = (*rg)(1);
  }
  collisions.clear();
}

bool CollisionChecker::operator()(const Connector::Edge &e) {
#ifdef NO_OBS
  return false;
#else
  auto path = e.path();
  bool collision = false;
  for(size_t i=1; i<path.size(); i++) {
    const auto &p0 = path[i-1];
    const auto &p1 = path[i];
    if(env.collide<0,1>(p0) || env.collide<0,1>(p1))
      collision = true;
    if(env.collide<0,1>(path[i-1],path[i]))
      collision = true;
    if(collision)
      break;
  }
  /*
    if(collision)
      collisions.push_back(std::make_tuple(path.front(),path.back()));
      */
  return collision;
#endif
}

CollisionTimeSpaceChecker::CollisionTimeSpaceChecker(Models::Integrator2DTrajectorySolver &solver, DynamicRobosoccer<scalar, obs_count> &env)
  : solver(solver), env(env)
{
  rg = new RandomGen<4,scalar>(
  {-SAMPLE_X0,-SAMPLE_X1,-SAMPLE_X2,-SAMPLE_X3},
  {SAMPLE_X0,SAMPLE_X1,SAMPLE_X2,SAMPLE_X3}
        );
}

void CollisionTimeSpaceChecker::setRandomObstacles() {
  for(auto& o : env.obs) {
    (*rg)(o);
  }
}

bool CollisionTimeSpaceChecker::operator()(const Connector::Edge &e) {
  auto collision = false;
  for(size_t i=1; i<e.size(); i++) {
    const auto &ts1 = e[i];
    const auto &ts0 = e[i-1];
    const auto &s1 = std::get<1>(ts1);
    const auto &t1 = std::get<0>(ts1);
    const auto &s0 = std::get<1>(ts0);
    const auto &t0 = std::get<0>(ts0);
    collision = env.collide<0,1>(s0,s1,t0,t1);
    if(collision) break;
  }
  return collision;
}

Connector::Connector(Models::Integrator2DTrajectorySolver &solver, TreeInt2D &tree)
  : solver(solver), tree(tree) {}

void TreeInt2D::reset()
{
  last_checked_idx = -1;
  tree.clear();
  parent.clear();
  trajectories.clear();
}

void TreeInt2D::dump_text(const string &node_file, const string &parent_file, const string &trajectory_file)
{
  Logger logger;
  for(const auto& n : tree.cloud.states)
    logger << "("
           << n(0) << "," << n(1) << ","
           << n(2) << "," << n(3) << ","
           << n.cost()
           << ")\n";
  logger.save(node_file);
  logger.clear();

  for(const auto &p : parent)
    logger << p << "\n";
  logger.save(parent_file);
  logger.clear();

  for(const auto &trj : trajectories) {
    for(size_t i=0; i<=segment; i++)
    {
      const auto &t = trj[i];
      auto p = std::get<1>(t);
      logger << "("
             << std::get<0>(t) << ","
             << p(0) << "," << p(1) << ","
             << p(2) << "," << p(3)
             << ") ";
    }
    logger << "\n";
  }
  logger.save(trajectory_file);
  logger.clear();
}

void TreeInt2D::from_text(const string &node_file, const string &parent_file, const string &trajectory_file)
{
  std::ifstream tree_stream(node_file);
  std::ifstream parent_stream(parent_file);
  std::ifstream trajectory_stream(trajectory_file);
  std::string line;

  tree.clear();
  parent.clear();
  trajectories.clear();

  while(std::getline(tree_stream, line)) {
    line.erase(std::remove(line.begin(), line.end(), '('), line.end());
    line.erase(std::remove(line.begin(), line.end(), ')'), line.end());
    std::string s;
    std::vector<std::string> values;
    std::istringstream ss(line);
    while(std::getline(ss,s,','))
      values.push_back(s);
    state_t state;
    // std::cout << line << std::endl;
    for(size_t i=0; i<4; i++)
      state[i] = std::stod(values[i]);
    state.setCost(std::stod(values.back()));
    tree.addPoint(state);
  }
  while(std::getline(parent_stream, line))
    parent.push_back(std::stoi((line)));
  while(std::getline(trajectory_stream, line)) {
    std::string s;
    std::vector<std::string> values;
    std::istringstream ss(line);
    Trajectory<scalar,state_t,segment> trajectory;
    while(std::getline(ss, s, ' '))
      values.push_back(s);
    for(size_t i=0; i<values.size(); i++) {
      auto& str = values.at(i);
      str.erase(std::remove(str.begin(), str.end(), '('), str.end());
      str.erase(std::remove(str.begin(), str.end(), ')'), str.end());
      std::string sstr;
      std::istringstream sss(str);
      std::vector<std::string> states;
      state_t state;
      while(std::getline(sss, sstr, ','))
        states.push_back(sstr);
      // std::cout << str << std::endl;
      auto time = std::stod(states.front());
      for(size_t k=1; k<=4; k++)
        state[k-1] = std::stold(states.at(k));
      trajectory.at(i) = std::make_tuple(time,state);
    }
    trajectories.push_back(trajectory);
  }

  tree_stream.close();
  parent_stream.close();
  trajectory_stream.close();
}

Trajectories<scalar, state_t, segment> TreeInt2D::get_trajectory(TreeInt2D::Index idx) {
  auto i = idx;
  Trajectories<scalar,state_t,segment> sol;
  while(i>0) {
    // sol.push_back(trajectories[i]);
    sol.insert(sol.begin(), trajectories[i]);
    i = parent[i];
  }
  return sol;
}

CostInt2D::CostInt2D(Models::Integrator2DTrajectorySolver &solver) : solver(solver) {}

CostInt2D::Scalar CostInt2D::operator()(const state_t &s0, const state_t &s1) const
{
  Models::Integrator2DSS::StateType xi;
  Models::Integrator2DSS::StateType xf;
  for(size_t i=0; i<4; i++) {
    xi(i) = s0(i);
    xf(i) = s1(i);
  }
  return std::get<1>(solver.cost(xi,xf));
}

TreeInt2D &Wrapper::get_tree_int2d()
{
  if(!tree_int2d) initialize();
  return *tree_int2d;
}

StaticEnvironment &Wrapper::get_robosoccer_env()
{
  if(!robosoccer_env) initialize();
  return *robosoccer_env;
}

DynamicEnvironment &Wrapper::get_dynamic_soccer_env()
{
  if(!dynamic_soccer_env) initialize();
  return *dynamic_soccer_env;
}

CostInt2D &Wrapper::get_cost_int2d()
{
  if(!cost_int2d) initialize();
  return *cost_int2d;
}

Connector &Wrapper::get_connector()
{
  if(!connector) initialize();
  return *connector;
}

GoalChecker<StaticEnvironment> &Wrapper::get_goal()
{
  if(!goal) initialize();
  return *goal;
}

GoalChecker<DynamicEnvironment> &Wrapper::get_goal_dynamic_env()
{
  if(!goal_dynamic_env) initialize();
  return *goal_dynamic_env;
}

Sampler<StaticEnvironment> &Wrapper::get_sampler()
{
  if(!sampler) initialize();
  return *sampler;
}

Sampler<DynamicEnvironment> &Wrapper::get_sampler_dynamic_env()
{
  if(!sampler_dynamic_env) initialize();
  return *sampler_dynamic_env;
}

NeighborRadius &Wrapper::get_radius()
{
  if(!radius) initialize();
  return *radius;
}

CollisionChecker &Wrapper::get_checker()
{
  if(!checker) initialize();
  return *checker;
}

CollisionTimeSpaceChecker &Wrapper::get_checker_time_space()
{
  if(!checker_time_space) initialize();
  return *checker_time_space;
}

RRTStarInt2D &Wrapper::get_rrtstar_int2d()
{
  if(!rrtstar_int2d) initialize();
  return *rrtstar_int2d;
}

RRTStarInt2DTimeSpaceObs &Wrapper::get_rrtstar_int2d_timespace_obs()
{
  if(!rrtstar_int2d_timespace_obs) initialize();
  return *rrtstar_int2d_timespace_obs;
}

void Wrapper::initialize()
{
  tree_int2d  = new TreeInt2D;
  robosoccer_env = new StaticEnvironment;
  dynamic_soccer_env = new DynamicEnvironment;
  cost_int2d = new CostInt2D(integrator2d.solver);
  connector = new Connector(integrator2d.solver, *tree_int2d);
  goal = new GoalChecker<StaticEnvironment>(*robosoccer_env);
  goal_dynamic_env = new GoalChecker<DynamicEnvironment>(*dynamic_soccer_env);
  sampler = new Sampler<StaticEnvironment>(*robosoccer_env);
  sampler_dynamic_env = new Sampler<DynamicEnvironment>(*dynamic_soccer_env);
  radius = new NeighborRadius;
  checker = new CollisionChecker(integrator2d.solver, *robosoccer_env);
  checker_time_space = new CollisionTimeSpaceChecker(integrator2d.solver, *dynamic_soccer_env);
  rrtstar_int2d = new RRTStarInt2D(*tree_int2d, *cost_int2d, *sampler, *checker, *radius, *goal, *connector);
  rrtstar_int2d_timespace_obs = new RRTStarInt2DTimeSpaceObs(*tree_int2d, *cost_int2d, *sampler_dynamic_env, *checker_time_space, *radius, *goal_dynamic_env, *connector);
}

Models::Integrator2D Wrapper::integrator2d;
TreeInt2D *Wrapper::tree_int2d;
StaticEnvironment *Wrapper::robosoccer_env;
DynamicEnvironment *Wrapper::dynamic_soccer_env;
CostInt2D *Wrapper::cost_int2d;
Connector *Wrapper::connector;
GoalChecker<StaticEnvironment> *Wrapper::goal;
GoalChecker<DynamicEnvironment> *Wrapper::goal_dynamic_env;
Sampler<StaticEnvironment> *Wrapper::sampler;
Sampler<DynamicEnvironment> *Wrapper::sampler_dynamic_env;
NeighborRadius *Wrapper::radius;
CollisionChecker *Wrapper::checker;
CollisionTimeSpaceChecker *Wrapper::checker_time_space;
RRTStarInt2D *Wrapper::rrtstar_int2d;
RRTStarInt2DTimeSpaceObs *Wrapper::rrtstar_int2d_timespace_obs;
