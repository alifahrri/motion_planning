#include <gtest/gtest.h>
#include "elements.hpp"
#include "sampler.hpp"

using namespace mpl;

template <size_t dim>
struct sample_t {
	double& operator()(size_t i) {
		return state[i];
	}
	const double& operator()(size_t i) const{
		return state[i];
	}
	double state[dim];
};

template <size_t dim, typename state_array, typename min_array, typename max_array>
inline
bool test(const state_array &state, const min_array &min, const max_array &max)
{
	using elements::element;
	auto ok = true;
	for(size_t i=0; i<dim; i++) {
		if((element(state, i) < element(min, i)) || (element(state, i) > element(max, i)))
			ok = false;
	}
	return ok;
}

TEST(sampler3f, sample1) {
	Sampler<3,double> sampler;
	auto min = std::vector<double>{0.0,0.0,0.0};
	auto max = std::vector<double>{1.0,1.0,1.0};
	auto sample = sampler.sample<sample_t<3>>();
	EXPECT_TRUE(test<3>(sample, min, max));
}

TEST(sampler3f, sample2) {
	Sampler<3,double> sampler;
	auto min = std::vector<double>{0.0,0.0,0.0};
	auto max = std::vector<double>{1.0,1.0,1.0};
	auto sample = sampler.sample<State<double,3>>();
	EXPECT_TRUE(test<3>(sample, min, max));
}

TEST(sampler3f, sample3) {
	Sampler<3,double> sampler;
	auto min = std::vector<double>{1.0,1.0,1.0};
	auto max = std::vector<double>{10.0,10.0,10.0};
	sampler.set_bound(min,max);
	auto sample = sampler.sample<State<double,3>>();
	EXPECT_TRUE(test<3>(sample,min,max));
}

TEST(sampler3f, sample4) {
	Sampler<3,double> sampler;
	auto min = std::vector<double>{-1.0,-1.0,-1.0};
	auto max = std::vector<double>{1.0,1.0,1.0};
	sampler.set_bound(min,max);
	auto sample = sampler.sample<State<double,3>>();
	EXPECT_TRUE(test<3>(sample,min,max));
}

TEST(sampler3f, sample5) {
	Sampler<3,double> sampler;
	auto min = std::vector<double>{-1.0,-1.0,-1.0};
	auto max = std::vector<double>{0.0,0.0,0.0};
	sampler.set_bound(min,max);
	auto sample = sampler.sample<State<double,3>>();
	EXPECT_TRUE(test<3>(sample,min,max));
}

TEST(sampler3f, sample6) {
	Sampler<3,double> sampler;
	auto min = std::vector<double>{-1.0,-1.0,-1.0};
	auto max = std::vector<double>{1.0,1.0,1.0};
	sampler.set_bound(min,max);
	auto sample = State<double,3>();
	sampler.sample<State<double,3>>(sample);
	EXPECT_TRUE(test<3>(sample,min,max));
}

TEST(sampler3f, sample7) {
	Sampler<3,double> sampler;
	auto min = std::vector<double>{-1.0,-1.0,-1.0};
	auto max = std::vector<double>{1.0,1.0,1.0};
	sampler.set_bound(min,max);
	auto sample = sampler();
	EXPECT_TRUE(test<3>(sample,min,max));
}

TEST(sampler3f, sample8) {
	Sampler<3,double> sampler;
	auto min = std::vector<double>{-1.0,-1.0,-1.0};
	auto max = std::vector<double>{1.0,1.0,1.0};
	sampler.set_bound(min,max);
	double sample[3];
	sampler(sample);
	EXPECT_TRUE(test<3>(sample,min,max));
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}

