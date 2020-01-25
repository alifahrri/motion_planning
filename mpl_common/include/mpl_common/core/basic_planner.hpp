#ifndef CORE_PLANNER_HPP
#define CORE_PLANNER_HPP

namespace mpl {
    namespace core {
        
        /**
         * @brief crtp base for sbmp
         * @tparam T derived class
         * @requirements 
         * - 
         */
        template <typename T>
        struct BasePlanner {
            T& self() {
                return *static_cast<T*>(this);
            }
            
            template <typename start_t, typename goal_t>
            bool plan(const start_t &start, const goal_t &goal, size_t max_iter) {
                /* TODO : fix member variable function call */
                self()._planner->setStart(start);
                self()._goal->set_goal(goal);
                // self()._planner->set_goal(goal);
                bool goal_found = false;
                for (size_t i=0; i<max_iter; i++)
                    goal_found = self()._planner->grow();
                return goal_found;
            }
        };
    } // namespace core
} // namespace mpl
#endif // CORE_PLANNER_HPP