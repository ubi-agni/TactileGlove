#pragma once

#include <string>
#include <values.h>

class Range {
public:
	Range (float fMin=FLT_MAX, float fMax=-FLT_MAX) 
		: fMin(fMin), fMax(fMax) {}

	float  min() const {return fMin;}
	float& min() {return fMin;}

	float  max() const {return fMax;}
	float& max() {return fMax;}

	float  range() const {return fMax-fMin;}

	void   update (float fValue) {
		if (fValue < fMin) fMin=fValue;
		if (fValue > fMax) fMax=fValue;
	}

private:
	float fMin, fMax;
};

/* Hardware abstraction of a single tactile sensing element.
	The class computes some internal data statistics to compute 
	values smoothed over time, current range statistics and derived values
*/
class TactileSensor {
public:
	enum Mode {rawCurrent = 0, // current raw value, now computation, no averinging
		        rawMean, // averaged raw value
		        absCurrent,  // raw value - min value
		        absMean, // averaged absCurrent
		        relCurrent,  // relative value (0-1) within min-max range
		        relMean, // averaged relCurrent
		        relCurrentRelease, // like relCurrent, but negative values indicate difference force to recently released grasp
		        relMeanRelease, // averaged relMeanRelease
		        lastMode};
	TactileSensor ();

	static char getModeID (const std::string& sName);
	static std::string getModeName (Mode m);

	void  update (float fNew);
	// normalize according to internal dynamic range for relative modes
	float value (Mode mode, float range=0.0) const;
	// considering given fRelMin as lower boundary for relative modes
	float value (Mode mode, float fMinRange, float fRelMin) const;
	// return sliding mean
	float mean () const {return fMean;}

	void  setMeanLambda (float fLambda);
	void  setRangeLambda (float fLambda);
	void  setReleaseDecay (float fDecay);

	float getMeanLambda () const {return fMeanLambda;}
	float getRangeLambda () const {return fRangeLambda;}
	float getReleaseDecay () const {return fReleaseDecay;}

	const Range& absRange () const {return rAbsRange;}
	const Range& dynRange () const {return rDynRange;}
	Range range () const;

protected:
	float fMeanLambda, fRangeLambda, fReleaseDecay;
	float fCur, fMean, fReleased;
	Range rAbsRange;
	Range rDynRange;
};
