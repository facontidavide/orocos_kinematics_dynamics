#ifndef FRAMEDIFF_TEST_HPP
#define FRAMEDIFF_TEST_HPP

#include <cppunit/extensions/HelperMacros.h>
#include <framesdiff.hpp>

using namespace KDL;

class FrameDiffTest : public CppUnit::TestFixture
{
    CPPUNIT_TEST_SUITE( FrameDiffTest);
    CPPUNIT_TEST( SimpleTest );
    CPPUNIT_TEST_SUITE_END();

public:
    void setUp();
    void tearDown();

    void SimpleTest();

};

#endif
