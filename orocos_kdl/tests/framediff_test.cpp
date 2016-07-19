#include "framediff_test.hpp"
#include <frames_io.hpp>
#include <cstdlib>
#include <math.h>

CPPUNIT_TEST_SUITE_REGISTRATION( FrameDiffTest );


using namespace KDL;
using namespace std;

double RandAngle()
{
    const long MAX = 100000.0;
    double r = static_cast<double>(std::rand()%(2*MAX) - MAX);
    r /= static_cast<double>( MAX );
    return (M_PI * r);
}

void FrameDiffTest::setUp()
{
    std::srand(std::time(0));
}

void  FrameDiffTest::tearDown()
{
}

void FrameDiffTest::SimpleAxisTest()
{
    for (int i=0; i<10; i++)
    {
        double roll  = RandAngle();
        double pitch = RandAngle();
        double yaw   = RandAngle();

        Rotation rotA = Rotation::RPY( roll, pitch, yaw );

        Rotation rotB( rotA );

        Vector test_vector[3];
        test_vector[0] = Vector(1,0,0);
        test_vector[1] = Vector(0,1,0);
        test_vector[2] = Vector(0,0,1);

        for (int v=0; v<3; v++)
        {
            {
                rotB = rotA;
                Vector difference = diff_2DoF( rotA, rotB, test_vector[v] );
                CPPUNIT_ASSERT_DOUBLES_EQUAL( difference(0), 0.0, 0.0001);
                CPPUNIT_ASSERT_DOUBLES_EQUAL( difference(1), 0.0, 0.0001);
                CPPUNIT_ASSERT_DOUBLES_EQUAL( difference(2), 0.0, 0.0001);
            }

            for (int t=0; t<3; t++)
            {
                double angle = RandAngle();

                rotB = ( rotA*Rotation::Rot(test_vector[t], angle) );

                Vector diff_5 = diff_2DoF( rotA, rotB, test_vector[v] );
                Vector diff_6 = diff( rotA, rotB );

                /*std::cout << "----------------------------\n" << std::endl;
                std::cout << rotA << std::endl;
                std::cout << rotB << std::endl;
                std::cout << "----\n" << std::endl;
                std::cout << diff_5 << std::endl;
                std::cout << diff_6 << std::endl;
                */
                if( t == v)
                {
                    CPPUNIT_ASSERT_DOUBLES_EQUAL( diff_5[0], 0, epsilon);
                    CPPUNIT_ASSERT_DOUBLES_EQUAL( diff_5[1], 0, epsilon);
                    CPPUNIT_ASSERT_DOUBLES_EQUAL( diff_5[2], 0, epsilon);
                }
                else{

                    CPPUNIT_ASSERT_DOUBLES_EQUAL( diff_5[0], diff_6[0], epsilon);
                    CPPUNIT_ASSERT_DOUBLES_EQUAL( diff_5[1], diff_6[1], epsilon);
                    CPPUNIT_ASSERT_DOUBLES_EQUAL( diff_5[2], diff_6[2], epsilon);
                }
            }
        }
    }

}

