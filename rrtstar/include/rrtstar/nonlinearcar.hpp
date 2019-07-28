#ifndef NONLINEARCARRRT_HPP
#define NONLINEARCARRRT_HPP

#include <cmath>
#include <memory>
#include <type_traits>

#include "sampler.hpp"
#include "kdtree.hpp"
#include "rrtstar.hpp"
#include "rrtvisual.hpp"
#include "models/dynamics/nonlinearcar.hpp"
#include "trajectory.hpp"
#include "random.hpp"
#include "environment.hpp"
#include "tree.hpp"

namespace Kinodynamic {
    namespace Car {

        // typedef double cost_t;
        // typedef Models::Scalar Scalar;
        // typedef State<Scalar,Models::n> CarState;
        // typedef States<Scalar,Models::n> CarStates;
        // typedef KDTree<states_t,Models::n,state_t> KDTreeCar;
        // typedef Models::NonlinearCarCost CostFN;

        constexpr int NSEGMENT = 10;
        constexpr auto N = Models::n;
        /* define types */
        using Scalar = Models::Scalar;
        using Cost = Scalar;
        using CarState = State<Scalar,N>;
        using CarStates = States<Scalar,N>;
        using CarTimeState = mpl::TimeState<Scalar,CarState>;
        using ModelState = Models::NonlinearCar::State;
        using CarTrajectory = mpl::Trajectory<CarTimeState,NSEGMENT>;
        using CarTree = mpl::Tree<N,Scalar,CarStates,CarTrajectory>;
        using CostFN = Models::NonlinearCarCost;
        using CarSampler = mpl::Sampler<N,Scalar>;
        using Environment = environment::Environment<>;
        using CarTrajectorySolver = Models::NonlinearCarTrajectorySolver;
        using CarConnector = mpl::Connector<CarTrajectory,CarTrajectorySolver,CarTree>;
        
        /* construct RRTStar Type */
        // using CarRRT = RRTStar<KDTreeCar,Cost,CarSampler,>;
    } // namespace Car
} // namespace Kinodynamic

#endif // NONLINEARCARRRT_HPP