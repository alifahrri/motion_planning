#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include "utility.hpp"
#include "core/basic_trajectory.hpp"
#include "core/basic_timestate.hpp"
#include "core/basic_connector.hpp"

#include <tuple>

namespace mpl {

namespace detail {
    template <typename Scalar>
    constexpr auto time_offset(auto &offset) 
        -> decltype(std::is_same<decltype(offset()),Scalar>::value,Scalar{}) 
    {
        return offset();
    }
    template <typename Scalar>
    constexpr auto time_offset(auto &offset) 
        -> decltype(std::is_same<decltype(offset.offset()),Scalar>::value,Scalar{}) 
    {
        return offset.offset();
    }
    template <typename Scalar>
    constexpr auto time_offset(auto &offset) 
        -> decltype(std::is_same<decltype(offset.time_offset()),Scalar>::value,Scalar{}) 
    {
        return offset.time_offset();
    }
    
    template <typename Connector>
    struct ConnectorCommon {
        constexpr auto last_connection() {
            return Connector::edge;
        }
    };
} // namespace detail 

/**
 * @brief : a type represents time-state tuple
 * @tparam : 
 * Scalar   = a type for time
 * State    = a type for state
 * @requirements :
 * - State has constexpr static member dim, represents state dimension
 * TODO : provide state dimension deductions, e.g. via std::size or std::tuple_size
 */
template<typename Scalar, typename State>
struct TimeState : core::BaseTimeState<TimeState<Scalar,State>>
{
  using state_type                      = State;
  using time_type                       = Scalar;
  using timestate_type                  = std::tuple<time_type,state_type>;
  constexpr static size_t state_dim     = State::dim;
  
  TimeState() {}
  /**
   * @brief : construct TimeState from std::tuple
   */
  TimeState(const timestate_type& rhs) {
    this->tuple = rhs;
  }
  /**
   * @brief : construct TimeState from generic type of time_t and state_t
   * @requirements :
   * - time_t is convertible to time_type
   * - state_t is convertible to state_type
   */
  template <typename time_t, typename state_t>
  TimeState(const time_t &t, const state_t &s) {
      this->tuple = std::make_tuple(
          time_type(t),state_type(s)
      );
  }
  /**
   * @brief : define copy from std::tuple
   */
  TimeState<Scalar,State>& operator= (const timestate_type& rhs) {
    std::get<0>(this->tuple) = std::get<0>(rhs);
    std::get<1>(this->tuple) = std::get<1>(rhs);
    return *this;
  }
  /* TODO : remove; Reason : ambiguous */
  template<typename Index>
  Scalar operator()(Index idx) const {
      return (idx < this->state_dim ? std::get<1>(this->tuple)(idx) : std::get<0>(this->tuple));
  }
  timestate_type tuple;
};

/**
 * @brief : a generic fixed size trajectory containing timestate
 * @tparam :
 * TimeState = type that represents time-stat tuple
 * n = length of trajectory
 * @requirements :
 * - TimeState has public member types called state_type
 * - TimeState has public member types called time_type
 * - TimeState is assignable from std::tuple< time_type, state_type >
 * - TimeState has mutable member function time()
 */
template<typename TimeState, int n>
struct Trajectory : core::BaseIterableTrajectory<Trajectory<TimeState,n>>
{
  using timestate_type                    = TimeState;
  using state_type                        = typename timestate_type::state_type;
  using time_type                         = typename timestate_type::time_type;
  using time_container_type               = std::array<time_type,n>;
  using state_container_type              = std::array<state_type,n>;
  using timestate_container_type          = std::array<timestate_type,n>; 
  using base_type                         = core::BaseIterableTrajectory<Trajectory<TimeState,n>>;
  constexpr static size_t state_dimension = timestate_type::StateDimension;
  
  /**
   * @brief : construct Trajectory from trajectory with different type
   * @tparam : trajectory_t type that represents trajectory
   * @requirements :
   * trajectory_t satisfy requirements from core::BaseIterableTrajectory::copy_from, that are :
   * - a call to std::size(trajectory_t{}) is valid
   * - trajectory_t has member function [] that returns time-state representation, say ts_repr_t
   * - a call to mpl::core::detail::get_time(ts_repr_t{}) is valid and its return value is Convertible to time_type
   * - a call to mpl::core::detail::get_state(ts_repr_t{}) is valid and its return value is Convertible to state_type
   */
  template <typename trajectory_t>
  Trajectory(const trajectory_t &t)
  { 
      base_type::copy_from(t); 
  }
  /**
   * @brief : construct Trajectory via copy
   */
  Trajectory(const Trajectory& t) {
    for(size_t i=0; i<n; i++)
      (*this)[i] = t[i];
  }
  /**
   * @brief
   */
  Trajectory() {
    for(size_t i=0; i<n; i++)
      (*this)[i] = std::make_tuple(time_type(0.0),state_type{});
  }
  Trajectory(const state_type &s) {
    for(size_t i=0; i<n; i++)
      (*this)[i] = std::make_tuple(time_type(0.0),s);
  }
  /**
   * @brief : shift trajectory's time by t
   */
  Trajectory<timestate_type,n> operator + (const time_type &t)
  {
    auto trajectory = *this;
    for(size_t i=0; i<this->size(); i++)
      this->operator[](i).time() += t;
    return trajectory;
  }
  timestate_container_type trajectory;
};

/**
 * @brief : partial template specialization of Trajectory class that has dynamic size trajectory
 * @tparam : 
 * TimeState = a type that represents time-state tuple
 * @requirements : same as above
 * TODO : specialize for any negative (and zero) value 
 */
template<typename TimeState>
struct Trajectory<TimeState,-1> : core::BaseIterableTrajectory<Trajectory<TimeState,-1>>
{
  using timestate_type              = TimeState;
  using state_type                  = typename timestate_type::state_type;
  using time_type                   = typename timestate_type::time_type;
  using timestate_container_type    = std::vector<timestate_type>;
  using time_container_type         = std::vector<time_type>;
  using state_container_type        = std::vector<state_type>;
  using base_type                   = core::BaseIterableTrajectory<Trajectory<TimeState,-1>>;
  using allocator_type              = typename timestate_container_type::allocator_type;
  
  template <typename trajectory_t>
    // requires Container<trajectory_t> // && is_tuple<trajectory_t.at(0)>
  Trajectory(const trajectory_t &t)
  {
    base_type::copy_from(t);
  }
  Trajectory(const Trajectory& t) {
      (*this) = t;
  }
  Trajectory() {}
  Trajectory(const state_type &s) {
    this->trajectory.push_back(std::make_tuple(time_type{0.0},s));
  }
  Trajectory<timestate_type,-1> operator + (const time_type &t)
  {
    auto trajectory = *this;
    for(auto & trj: trajectory)
      trj.time() += t;
    return trajectory;
  }
  
  timestate_container_type trajectory;
};

/**
 * @brief : a generic class that store latest state, solves, shifts trajectory
 * @tparam :
 * Trajectory   = a type that stores trajectory (or edge) information
 * Solver       = a type that solves trajectory from initial state to final state
 * @requirements :
 * Solver::State is defined,
 * Trajectory::state_type is defined,
 * Trajectory::time_type is defined
 * @todo : use Solver::state_type instead of Solver::State
 */
template <typename Trajectory, typename Solver, typename TimeOffset>
struct Connector : detail::ConnectorCommon<Connector<Trajectory,Solver,TimeOffset>>, core::BaseConnetor<Connector<Trajectory,Solver,TimeOffset>>
{
  using edge_type           = Trajectory;
  using solver_type         = Solver;
  using solver_state_type   = typename solver_type::State;
  using edge_state_type     = typename edge_type::state_type;
  using edge_time_type      = typename edge_type::time_type;

  Connector(solver_type &solver, TimeOffset &time_offset)
    : solver(solver)
    , time_offset(time_offset)
  {}
  
  /*
  template <typename trajectory_t, typename state_t>
    // requires Convertible<state_t,SolverState> && Convertible<SolverTrajectory,trajectory_t> && DefaultConstructible<trajectory_t>
  trajectory_t operator()(const state_t &s0, const state_t &s1, size_t n) {
      trajectory_t trajectory;
      auto result = solver.solve(
          solver_state_type(s0),solver_state_type(s1),n
      );
      trajectory = result;
      return trajectory;
  }
  
  template <typename trajectory_t, typename state_t, size_t nsegment>
    // requires Convertible<state_t,SolverState> && Convertible<SolverTrajectory,trajectory_t> && DefaultConstructible<trajectory_t>
  trajectory_t operator()(const state_t &s0, const state_t &s1) {
      trajectory_t trajectory;
      auto result = solver.template solve<nsegment-1>(
          solver_state_type(s0),solver_state_type(s1)
      );
      trajectory = result;
      return trajectory;
  }
  */
  
  /*
  template <typename state_t, size_t nsegment=10>
    // requires Convertible<state_t,SolverState>
  auto operator()(const state_t &s0, const state_t &s1) {
    auto trajectory = solver.template solve<nsegment-1>(
        solver_state_type(s0),
        solver_state_type(s1)
    );
    edge_time_type t = detail::time_offset<edge_time_type>(time_offset);
    edge = edge_type(trajectory);
    edge = edge + t;
    return edge;
  }
  */
  
  /*
  template <size_t nsegment>
  constexpr auto operator()(const solver_state_type &s0, const solver_state_type &s1) {
    auto trajectory = solver.template solve<nsegment-1>(s0, s1);
    edge_time_type t = detail::time_offset<edge_time_type>(time_offset);
    edge = edge_type(trajectory);
    edge = edge + t;
    return edge;
  }
  */

  edge_type edge;
  solver_type &solver;
  TimeOffset &time_offset;
};

} // namespace mpl

#endif // TRAJECTORY_HPP