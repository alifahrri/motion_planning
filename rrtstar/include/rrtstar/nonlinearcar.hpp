#ifndef NONLINEARCARRRT_HPP
#define NONLINEARCARRRT_HPP

#include <cmath>
#include <memory>
#include <type_traits>

#include "tree.hpp"
#include "goal.hpp"
#include "kdtree.hpp"
#include "random.hpp"
#include "sampler.hpp"
#include "rrtstar.hpp"
#include "neighbor.hpp"
#include "rrtvisual.hpp"
#include "trajectory.hpp"
#include "environment.hpp"
#include "core/basic_planner.hpp"
#include "models/dynamics/nonlinearcar.hpp"

namespace Kinodynamic {
    namespace Car {

#ifndef NSEGMENT
        constexpr int NSEGMENT      = 10;
#endif
        constexpr auto N            = Models::n;
        
        /* TODO : omit 'Car', reason: redundant with namespace */
        /* define types */
        using Scalar                = Models::Scalar;
        using Cost                  = Scalar;
        using CarState              = State<Scalar,N>;
        using CarStates             = States<Scalar,N>;
        using CarTimeState          = mpl::TimeState<Scalar,CarState>;
        using ModelState            = Models::NonlinearCar::State;
        using CarTrajectory         = mpl::Trajectory<CarTimeState,NSEGMENT>;
        
        /* define necessary classes for rrt */
        using CarModel              = Models::NonlinearCar;
        using CarTrajectorySolver   = Models::NonlinearCarTrajectorySolver;
        using CarTree               = mpl::Tree<N,Scalar,CarStates,CarTrajectory>;
        using CostFN                = mpl::Cost<Scalar,CarState,CarTrajectorySolver>;
        using CarSampler            = mpl::Sampler<N,Scalar>;
        using Environment           = environment::Environment<>;
        using CarConnector          = mpl::Connector<CarTrajectory,CarTrajectorySolver,CarTree>;
        using CarNeighbor           = mpl::NeighborRadius<size_t,Scalar>;
        using CarGoal               = mpl::GoalChecker<CarState,mpl::DummyCheck>;
        
        /* construct RRTStar Type */
        using CarRRT                = RRTStar<CarTree,Cost,CarSampler,CarNeighbor,Environment,CarGoal,CarConnector>;
    } // namespace Car
    
    /**
     * @brief define kinodynamic rrt planner with nonlinearcar model
     */
    struct CarRRTPlanner : mpl::core::BasePlanner<CarRRTPlanner> {
        template <typename T>
        using ptr = std::unique_ptr<T>;
        CarRRTPlanner (){
            std::array<double,5> workspace;
            /* initialization */
            _model          = std::make_unique<Car::CarModel>();
            _tree           = std::make_unique<Car::CarTree>();
            _sampler        = std::make_unique<Car::CarSampler>();
            _environment    = std::make_unique<Car::Environment>();
            _cost           = std::make_unique<Car::CostFN>(_model->solver);
            // _solver         = std::make_unique<Car::CarTrajectorySolver>();
            _connector      = std::make_unique<Car::CarConnector>(_model->solver, *_tree);
            _neighbor       = std::make_unique<Car::CarNeighbor>(workspace);
            _goal           = std::make_unique<Car::CarGoal>();
            _planner        = std::make_unique<Car::CarRRT>(
                *_tree, *_cost, *_sampler, *_neighbor, 
                *_environment, *_goal, *_connector
            );
        }
        
        /* member variable */
        ptr<Car::CarModel>       _model;
        ptr<Car::CostFN>         _cost;
        ptr<Car::CarTree>        _tree;
        ptr<Car::CarSampler>     _sampler;
        ptr<Car::Environment>    _environment;
        // ptr<Car::CarTrajectorySolver> _solver;
        ptr<Car::CarConnector>   _connector;
        ptr<Car::CarNeighbor>    _neighbor;
        ptr<Car::CarGoal>        _goal;
        /* planner */
        ptr<Car::CarRRT>         _planner;
    };
} // namespace Kinodynamic

#endif // NONLINEARCARRRT_HPP