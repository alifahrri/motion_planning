#ifndef SAMPLER_HPP
#define SAMPLER_HPP

#include <type_traits>
#include <vector>
#include <array>

#include "states.hpp"
#include "random.hpp"
#include "elements.hpp"

namespace mpl {
	
	// generic sampler, scalable at runtime
template <size_t dim, typename scalar_t
		// base_container was used for default type of return value of operator ()
		, typename base_container = std::array<scalar_t, dim>
		// TODO : implements rejection method
		, typename rejection_t = nullptr_t>
struct Sampler : RandomGen<dim,scalar_t> {
	Sampler() 
		: RandomGen<dim,scalar_t>()
	{
		std::vector<scalar_t> lower(dim, scalar_t(0.0));
		std::vector<scalar_t> upper(dim, scalar_t(1.0));
		set_bound(lower, upper);
	}
	
	template <typename state_t>
	inline
	state_t sample() 
	{
		state_t state;
		this->operator()(state);
		return state;
	}
	
	template <typename state_t>
	inline
	void sample(state_t &state) 
	{
		this->operator()(state);
	}
	
	template <typename array_like_t>
	inline
	void operator() (array_like_t &array) {
		using elements::element;
		auto states = RandomGen<dim,scalar_t>::operator()();
		for(size_t i=0; i<dim; i++) {
			element(array, i) = element(states, i) * element(scale, i) + element(shift, i);
		}
	}
	
	template <typename array_like_t>
	inline
	auto operator()() -> decltype(array_like_t{}) {
		using elements::element;
		auto ret = array_like_t{};
		(*this)(ret);
		return ret;
	}
	
	inline
	auto operator()() -> decltype(base_container{}) {
		return this->operator()<base_container>();
	}
	
	template <typename array_like_t>
	void set_bound(const array_like_t &min, const array_like_t &max) {
		using elements::element;
		for(size_t i=0; i<dim; i++) {
			element(lower_bound, i) = element(min, i);
			element(upper_bound, i) = element(max, i);
			element(scale, i) = element(upper_bound, i) - element(lower_bound, i);
			element(shift, i) = element(lower_bound, i);
		}
	}
	
	std::array<scalar_t, dim> lower_bound, upper_bound;
	std::array<scalar_t, dim> scale, shift;
};

}

#endif // SAMPLER_HPP