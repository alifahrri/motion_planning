#include <gtest/gtest.h>
#include "test_data.h"

// using namespace test_data;

int main(int argc, char **argv)
{
	testing::InitGoogleTest(&argc,argv);
	// test_data::init();
	return RUN_ALL_TESTS();
}
