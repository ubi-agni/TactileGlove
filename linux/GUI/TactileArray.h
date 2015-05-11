#pragma once

#include <stddef.h>
#include <vector>
#include <assert.h>
#include "TactileSensor.h"

/* Hardware abstraction for an array of tactile sensing elements (tactels).
	This adds several accumulation modes to aggregate all sensor values within the array
	into a single value.
 */
class TactileArray {
public: 
	typedef std::vector<float> vector_data;
	typedef std::vector<TactileSensor>::const_iterator const_iterator;
	typedef std::vector<TactileSensor>::iterator iterator;
	enum AccMode {Sum=0, 
					  SumPositive, SumNegative, 
					  CountPositive, CountNegative, 
					  Min, Max, 
					  lastMode};

	TactileArray (size_t n);

	static char getModeID (const std::string& sName);
	static std::string getModeName (AccMode m);

	size_t size() const {return vSensors.size();}
	const TactileSensor& operator[](size_t i) const 
		{return vSensors[i];}

	const_iterator begin() const {return vSensors.begin();}
	const iterator begin() {return vSensors.begin();}
	const_iterator end() const {return vSensors.end();}
	const iterator end() {return vSensors.end();}

	// update from all values in consecutive order from it to end
	template <class T>
	void updateValues(T it, T end);

	template <typename T>
	void copyValues(T v, TactileSensor::Mode mode, float fMinRange=0) const {
		fMinRange = std::max(fMinRange, fMax-fMin);
		for (const_iterator it=begin(), e=end(); it!=e; ++it, ++v)
			*v = it->value(mode, fMinRange);
	}

	vector_data getAbsValues () const;
	vector_data getValues (TactileSensor::Mode mode, 
	                       float fMinRange=0) const;
	static float accumulate (const vector_data& data, 
	                         AccMode mode=Sum, bool bMean=true);

	float min() const {return fMin;}
	float max() const {return fMax;}

	void setMeanLambda (double fLambda);
	void setRangeLambda (double fLambda);
	void setReleaseDecay (double fDecay);

	float getMeanLambda () const {return vSensors[0].getMeanLambda();}
	float getRangeLambda () const {return vSensors[0].getRangeLambda();}
	float getReleaseDecay () const {return vSensors[0].getReleaseDecay();}

private:
	std::vector<TactileSensor> vSensors;
	float fMin, fMax;
};

template <class T>
void TactileArray::updateValues(T v, T end) {
	assert(end-v <= (std::ptrdiff_t) vSensors.size());
	if (end-v == 0) return;

	iterator it=this->begin(), e=this->end();
	// update values and compute new minimum / maximum
	it->update(*v); // first value
	const Range& r = it->absRange();
	fMin=r.min(); fMax=r.max();

	if (end-v > 1) {
		// consider all remaining elements
		for (++it, ++v; v != end; ++it, ++v) {
			it->update(*v);
			const Range& r = it->absRange();
			fMin = std::min(fMin, r.min());
			fMax = std::max(fMax, r.max());
		}
	} else {
		// only a single value was provided: copy *v to all entries
		for (++it; it != e; ++it) {
			it->update(*v);
		}
	}
}
