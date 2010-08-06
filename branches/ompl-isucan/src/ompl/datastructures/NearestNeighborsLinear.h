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

#ifndef OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_LINEAR_
#define OMPL_DATASTRUCTURES_NEAREST_NEIGHBORS_LINEAR_

#include "ompl/datastructures/NearestNeighbors.h"
#include "ompl/util/Exception.h"
#include <algorithm>

namespace ompl
{

    template<typename _T>
    class NearestNeighborsLinear : public NearestNeighbors<_T>
    {
    public:
        NearestNeighborsLinear(void) : NearestNeighbors<_T>(), removed_(0)
	{
	}
	
	virtual ~NearestNeighborsLinear(void)
	{
	}
	
	virtual void clear(void)
	{
	    data_.clear();
	    active_.clear();
	    removed_ = 0;
	}

	virtual void add(_T &data)
	{
	    data_.push_back(data);
	    active_.push_back(true);
	}

	virtual bool remove(_T &data)
	{
	    for (int i = data_.size() - 1 ; i >= 0 ; --i)
		if (data_[i] == data)
		{
		    active_[i] = false;
		    removed_++;
		    return true;
		}
	    return false;
	}
	
	virtual _T nearest(const _T &data) const
	{
	    int pos = -1;
	    double dmin = 0.0;
	    for (unsigned int i = 0 ; i < data_.size() ; ++i)
	    {
		if (active_[i])
		{
		    double distance = NearestNeighbors<_T>::distFun_(data_[i], data);
		    if (pos < 0 || dmin > distance)
		    {
			pos = i;
			dmin = distance;
		    }
		}
	    }
	    if (pos >= 0) 
		return data_[pos];
	    
	    throw Exception("No elements found");
	}

	virtual void nearest(const _T &data, unsigned int k, std::vector<_T> &nbh) const
	{
	    nbh.clear();
	    for (unsigned int i = 0 ; i < data_.size() ; ++i)
		if (active_[i])
		    nbh.push_back(data_[i]);
	    std::sort(nbh.begin(), nbh.end(), MySort(data, NearestNeighbors<_T>::distFun_));
	    if (nbh.size() > k)
		nbh.resize(k);
	}
	
	virtual unsigned int size(void) const
	{
	    return data_.size() - removed_;
	}
	
	virtual void list(std::vector<_T> &data) const
	{
	    data.clear();
	    data.reserve(data_.size() - removed_);
	    for (unsigned int i = 0 ; i < data_.size() ; ++i)
		if (active_[i])
		    data.push_back(data_[i]);
	}
	
    protected:
	
	std::vector<_T>   data_;
	std::vector<bool> active_;
	unsigned int      removed_;
	
    private:
	
	struct MySort
	{
	    MySort(const _T &e, const typename NearestNeighbors<_T>::DistanceFunction &df) : e_(e), df_(df)
	    {
	    }
	    
	    bool operator()(const _T &a, const _T &b) const
	    {
		return df_(a, e_) < df_(b, e_);
	    }

	    const _T                                              &e_;
	    const typename NearestNeighbors<_T>::DistanceFunction &df_;
	};

    };
    
    
}

#endif
