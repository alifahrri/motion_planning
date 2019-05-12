#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

namespace mpl {

template<typename Scalar, typename State, size_t dim>
struct TimeState : std::tuple<Scalar,State>
{
  TimeState() {}
  TimeState(const std::tuple<Scalar,State>& rhs) {
    *this = rhs;
  }
  TimeState<Scalar,State,dim> &operator =(const std::tuple<Scalar,State>& rhs) {
    std::get<0>(*this) = std::get<0>(rhs);
    std::get<1>(*this) = std::get<1>(rhs);
    return *this;
  }

  template<typename Index, size_t time_idx = dim>
  Scalar operator()(Index idx) const {
    return (idx < time_idx ? std::get<1>(*this)(idx) : std::get<0>(*this));
  }
};

template<typename Scalar, typename State, int n, size_t dim>
struct Trajectory : std::array<TimeState<Scalar,State,dim>,n+1>
{
  template <typename TrajectorySolver>
  Trajectory(const TrajectorySolver &trajectory)
  {
    for(size_t i=0; i<=n; i++)
      (*this)[i] = std::make_tuple(Scalar(std::get<0>(trajectory[i])),State(std::get<1>(trajectory[i])));
  }
  Trajectory(const Trajectory& t) {
    for(size_t i=0; i<=n; i++)
      (*this)[i] = t[i];
  }
  Trajectory() {
    for(size_t i=0; i<=n; i++)
      (*this)[i] = std::make_tuple(Scalar(0.0),State());
  }
  Trajectory(const State &s) {
    for(size_t i=0; i<=n; i++)
      (*this)[i] = std::make_tuple(Scalar(0.0),s);
  }
  std::array<Scalar,n+1> time() const
  {
    std::array<Scalar,n+1> ret;
    for(size_t i=0; i<=n; i++)
      ret[i] = std::get<0>((*this)[i]);
    return ret;
  }
  Trajectory<Scalar,State,n,dim> operator + (const Scalar &t)
  {
    auto trajectory = *this;
    for(auto & trj: trajectory)
      std::get<0>(trj) += t;
    return trajectory;
  }
  std::array<State,n+1> path() const {
    std::array<State,n+1> ret;
    for(size_t i=0; i<=n; i++)
      ret[i] = std::get<1>((*this)[i]);
    return ret;
  }
  // const std::tuple<Scalar,State>& operator[](size_t i) const { return trj[i]; }
  // std::tuple<Scalar,State>& operator[](size_t i) { return trj[i]; }
  // std::array<std::tuple<Scalar,State>,n+1> trj;
};

// Trajectory with dynamic allocator (resizeable vector)
template<typename Scalar, typename State, size_t dim>
struct ResizeableTrajectory : std::vector<TimeState<Scalar,State,dim>>
{
  template <typename TrajectorySolver>
  ResizeableTrajectory(const TrajectorySolver &trajectory)
  {
    for(const auto &t : trajectory){
      auto ts = TimeState<Scalar,State,dim>(std::make_tuple(Scalar(std::get<0>(t)),State(std::get<1>(t))));
      this->push_back(ts);
    }
  }
  ResizeableTrajectory(const ResizeableTrajectory& t) {
    // this->insert(this->begin(),t.begin(),t.end());
    for(const auto &ts : t)
      this->push_back(ts);
  }
  ResizeableTrajectory() {

  }
  auto time() const
  {
    std::vector<Scalar> ret;
    for(const auto &t : *this)
      ret.push_back(std::get<0>(t));
    return ret;
  }
  auto operator + (const Scalar &t)
  {
    auto trajectory = *this;
    for(auto & trj: trajectory)
      std::get<0>(trj) += t;
    return trajectory;
  }
  auto path() const {
    std::vector<State> ret;
    for(const auto &t : *this)
      ret.push_back(std::get<1>(t));
    return ret;
  }
  auto insert(const auto &trajectory) {
    auto vtrajectory = ResizeableTrajectory(trajectory);
    auto tshift = (this->size() ? std::get<0>(this->back()) : 0.0);
    auto shifted_trajectory = vtrajectory+tshift;
    // this->insert(this->end(),shifted_trajectory.begin(),shifted_trajectory.end());
    for(const auto &ts : shifted_trajectory)
      this->push_back(ts);
  }
  // const std::tuple<Scalar,State>& operator[](size_t i) const { return trj[i]; }
  // std::tuple<Scalar,State>& operator[](size_t i) { return trj[i]; }
  // std::array<std::tuple<Scalar,State>,n+1> trj;
};

template <size_t dim, size_t nsegment, typename Scalar, typename State, typename Solver, typename Tree>
struct Connector
{
  using Edge = Trajectory<Scalar,State,nsegment,dim>;

  Connector(Solver &solver, Tree &tree)
    : solver(solver)
    , tree(tree)
  {}

  template <typename state_t>
  constexpr auto operator()(const state_t &s0, const state_t &s1) {
    using solver_state_t = typename Solver::State;
    auto trajectory = solver.solve<nsegment>(
      solver_state_t(s0),
      solver_state_t(s1)
    );
    Scalar t = 0.0;
    edge = decltype(edge)(trajectory);
    if(tree.last_checked_idx) {
      edge = edge + std::get<0>(
        tree.trajectories.at(tree.last_checked_idx).back()
      );
    }
    return edge;
  }

  constexpr auto last_connection() {
    return edge;
  }

  Edge edge;
  Solver &solver;
  Tree &tree;
};

} // namespace mpl

#endif // TRAJECTORY_HPP