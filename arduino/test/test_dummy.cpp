#include <gtest/gtest.h>
#include <stepper_lib.hpp>

// Write a test for the add function
TEST(StepperLib, Add)
{
    EXPECT_EQ(add(1, 2), 3);
    EXPECT_EQ(add(0, 0), 0);
    EXPECT_EQ(add(255, 1), 0);
    EXPECT_EQ(add(255, 2), 1);
}


int main(int argc, char **argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    // if you plan to use GMock, replace the line above with
    // ::testing::InitGoogleMock(&argc, argv);

    if (RUN_ALL_TESTS())
    ;

    // Always return zero-code and allow PlatformIO to parse results
    return 0;
}
