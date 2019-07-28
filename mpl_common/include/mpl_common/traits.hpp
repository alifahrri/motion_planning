#ifndef MPL_TRAITS_HPP
#define MPL_TRAITS_HPP

#include <cstdlib>
#include <type_traits>

namespace mpl {
    namespace traits {
        template <typename C, typename = void>
        struct is_dynamic_size_container : std::false_type {};
        
        /* let's define that dynamic size container has insert member function */
        template <typename C>
        struct is_dynamic_size_container<C,std::void_t<
            decltype(std::declval<C>().insert(std::declval<C>().begin(),std::declval<typename C::value_type>()))
        > /* void_t */ > : std::true_type {};
        
        template <typename T, typename = void>
        struct has_trajectory : std::false_type {};
        
        template <typename T>
        struct has_trajectory<T, std::void_t<
            decltype(std::declval<T>().trajectory)
        > /* void_t */ > : std::true_type {};
        
        template <typename T, typename = void>
        struct has_resize_op : std::false_type {};
        
        template <typename T>
        struct has_resize_op<T, std::void_t<
            decltype(std::declval<T>().resize(size_t{}))
        > /* void_t */ > : std::true_type {};
        
        template <typename T, typename = void>
        struct has_push_back_op : std::false_type {};
        
        template <typename T>
        struct has_push_back_op<T, std::void_t<
            decltype(std::declval<T>().push_back(std::declval<typename T::value_type>()))
        > /* void_t */ > : std::true_type {};
    } // namespace traits
} // namespace mpl
#endif // MPL_TRAITS_HPP