#ifndef RANDOM_HPP
#define RANDOM_HPP

#include <cstdlib>
#include <random>
#include <array>

template<size_t dim, typename scalar = double>
struct RandomGen
{
  RandomGen& operator = (const RandomGen &rhs) {
    for(size_t i=0; i<dim; i++) {
      if(dist[i]) *dist[i] = rhs.dist[i];
      else dist[i] = new std::uniform_real_distribution<>(rhs.dist[i]->a(),rhs.dist[i]->b());
      twister[i] = rhs.twister[i];
    }
    return *this;
  }

  RandomGen(std::initializer_list<scalar> min, std::initializer_list<scalar> max) 
	{
   _initialize(min, max);
  }
	
	RandomGen() 
	{
		std::vector<scalar> min(dim, scalar(0.0));
		std::vector<scalar> max(dim, scalar(1.0));
		_initialize(min, max);
	}
	
	template <typename array_like_t>
	void _initialize(const array_like_t &min, const array_like_t &max) {
    std::random_device rd;

    auto min_it = min.begin();
    auto max_it = max.begin();
    for(size_t i=0; i<dim; i++) {
      a[i] = *min_it++;
      if(min_it == min.end()) break;
    }

    for(size_t i=0; i<dim; i++) {
      b[i] = *max_it++;
      if(min_it == max.end()) break;
    }

    for(size_t i=0; i<dim; i++) {
      twister[i] = std::mt19937_64(rd());
      dist[i] = new std::uniform_real_distribution<>(a[i],b[i]);
    }
  }

  std::array<scalar,dim> min() { return a; }
  std::array<scalar,dim> max() { return b; }

  template <typename ArrayLike>
  inline
  void operator ()(ArrayLike &array) {
    for(size_t i=0; i<dim; i++)
      array[i] = (*dist[i])(twister[i]);
  }

  inline
  scalar operator()(size_t i) {
    if(i < dim) return (*dist[i])(twister[i]);
    else return scalar(0);
  }

  inline
  std::array<scalar,dim> operator ()() {
    std::array<scalar,dim> ret;
    (*this)(ret);
    return ret;
  }

  // related to min, max
  std::array<scalar,dim> a;
  std::array<scalar,dim> b;
  // twister engine
  std::mt19937_64 twister[dim];
  // distribution
  std::uniform_real_distribution<> *dist[dim];
};

// partial specialization for boolean
template<size_t dim>
struct RandomGen<dim,bool>
{
  RandomGen& operator = (const RandomGen &rhs) {
    for(size_t i=0; i<dim; i++) {
      if(dist[i]) *dist[i] = rhs.dist[i];
      else dist[i] = new std::bernoulli_distribution(rhs.dist[i]->p());
      twister[i] = rhs.twister[i];
    }
    return *this;
  }

  RandomGen(std::initializer_list<double> prob) {
    std::random_device rd;
    auto prob_it = prob.begin();
    for(size_t i=0; i<dim; i++) {
      p[i] = *prob_it++;
      if(prob_it == prob.end()) break;
    }

    for(size_t i=0; i<dim; i++) {
      twister[i] = std::mt19937_64(rd());
      dist[i] = new std::bernoulli_distribution(p[i]);
    }
  }

  std::array<double,dim> prob() { return p; }

  template <typename ArrayLike>
  inline
  void operator ()(ArrayLike &array) {
    for(size_t i=0; i<dim; i++)
      array[i] = (*dist[i])(twister[i]);
  }

  inline
  bool operator()(size_t i) {
    if(i < dim) return (*dist[i])(twister[i]);
    else return false;
  }

  inline
  std::array<bool,dim> operator ()() {
    std::array<bool,dim> ret;
    (*this)(ret);
    return ret;
  }

	// probability of success
  std::array<double,dim> p;
	// twister engine
  std::mt19937_64 twister[dim];
	// distribution
  std::bernoulli_distribution *dist[dim];
};

// partial specialization for int
template<size_t dim>
struct RandomGen<dim, int>
{
	typedef int scalar;
  RandomGen& operator = (const RandomGen &rhs) {
    for(size_t i=0; i<dim; i++) {
      if(dist[i]) *dist[i] = rhs.dist[i];
      else dist[i] = new std::uniform_int_distribution<>(rhs.dist[i]->a(),rhs.dist[i]->b());
      twister[i] = rhs.twister[i];
    }
    return *this;
  }

  RandomGen(std::initializer_list<int> min, std::initializer_list<int> max) 
	{
   _initialize(min, max);
  }
	
	RandomGen() 
	{
		std::vector<scalar> min(dim, scalar(0.0));
		std::vector<scalar> max(dim, scalar(1.0));
		_initialize(min, max);
	}
	
	template <typename array_like_t>
	void _initialize(const array_like_t &min, const array_like_t &max) {
    std::random_device rd;

    auto min_it = min.begin();
    auto max_it = max.begin();
    for(size_t i=0; i<dim; i++) {
      a[i] = *min_it++;
      if(min_it == min.end()) break;
    }

    for(size_t i=0; i<dim; i++) {
      b[i] = *max_it++;
      if(min_it == max.end()) break;
    }

    for(size_t i=0; i<dim; i++) {
      twister[i] = std::mt19937_64(rd());
      dist[i] = new std::uniform_real_distribution<>(a[i],b[i]);
    }
  }
	
  std::array<scalar,dim> min() { return a; }
  std::array<scalar,dim> max() { return b; }

  template <typename ArrayLike>
  inline
  void operator ()(ArrayLike &array) {
    for(size_t i=0; i<dim; i++)
      array[i] = (*dist[i])(twister[i]);
  }

  inline
  int operator()(size_t i) {
    if(i < dim) return (*dist[i])(twister[i]);
    else return scalar(0);
  }

  inline
  std::array<int,dim> operator ()() {
    std::array<int,dim> ret;
    (*this)(ret);
    return ret;
  }

  std::array<scalar,dim> a;
  std::array<scalar,dim> b;
  std::mt19937_64 twister[dim];
  std::uniform_int_distribution<> *dist[dim];
};

#endif // RANDOM_HPP
