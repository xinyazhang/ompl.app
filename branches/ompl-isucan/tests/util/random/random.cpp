/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */

#include <gtest/gtest.h>
#include "ompl/util/RandomNumbers.h"
#include <vector>

using namespace ompl;

/* Just test we get some random values */
TEST(Random, DifferentSeeds)
{
    RNG r1, r2, r3, r4;
    int same = 0;
    int eq   = 0;
    const int N = 100;
    for (int i = 0 ; i < N ; ++i)
    {
	int v1 = r1.uniformInt(0, 100);
	int v2 = r2.uniformInt(0, 100);
	int v3 = r3.uniformInt(0, 100);
	int v4 = r4.uniformInt(0, 100);
	printf("%d %d %d %d\n", v1, v2, v3, v4);
	
	if (v1 == v2 && v2 == v3 && v3 == v4)
	    eq++;
	if (v1 == r1.uniformInt(0, 100))
	    same++;
	if (v2 == r2.uniformInt(0, 100))
	    same++;
	if (v3 == r3.uniformInt(0, 100))
	    same++;
	if (v4 == r4.uniformInt(0, 100))
	    same++;
    }
    EXPECT_FALSE(eq > N / 2);
    EXPECT_TRUE(same < 2 * N);
}

TEST(Random, ValidRangeInts)
{
    RNG r;  
    const int N = 100;
    const int V = 10000 * N;
    std::vector<int> c(N + 1, 0);
    for (int i = 0 ; i < V ; ++i)
    {
	int v = r.uniformInt(0, N);
	EXPECT_TRUE(v >= 0);
	EXPECT_TRUE(v <= N);
	c[v]++;
    }
    
    for (unsigned int i = 0 ; i < c.size() ; ++i)
	EXPECT_TRUE(c[i] > V/N/3);
}

int main(int argc, char **argv)
{
    ompl::RNG::setSeed(1);
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
