#include "TactileSensor.h"
#include <math.h>

TactileSensor::TactileSensor () 
	: fMeanLambda (0.7), fRangeLambda (0.9995), fReleaseDecay (0.05),
	  fCur(0), fMean(0), fReleased(FLT_MAX) // release mode not active
{
}

char TactileSensor::getModeID (const std::string& sName) {
	if (sName == "default") return absCurrent;
	else if (sName == "rawCurrent") return rawCurrent;
	else if (sName == "rawMean") return rawMean;
	else if (sName == "absCurrent") return absCurrent;
	else if (sName == "absMean") return absMean;
	else if (sName == "dynCurrent") return dynCurrent;
	else if (sName == "dynMean") return dynMean;
	else if (sName == "dynCurrentRelease") return dynCurrentRelease;
	else if (sName == "dynMeanRelease") return dynMeanRelease;
	return -1;
}
std::string TactileSensor::getModeName (Mode m) {
	switch (m) {
		case rawCurrent: return "rawCurrent";
		case rawMean: return "rawMean";
		case absCurrent: return "absCurrent";
		case absMean: return "absMean";
		case dynCurrent: return "dynCurrent";
		case dynMean: return "dynMean";
		case dynCurrentRelease: return "dynCurrentRelease";
		case dynMeanRelease: return "dynMeanRelease";
		default: return "";
	}
}

void TactileSensor::init(float fMin, float fMax)
{
	rAbsRange.min() = fMin;
	rAbsRange.max() = fMax;
}

void TactileSensor::update (float fNew) {
	if (!isfinite(fNew)) return; // do not use invalid value
	fMean = fNew + fMeanLambda * (fMean - fNew);

	rAbsRange.update (fNew); // set all-time minimum + maximum
	rDynRange.update (fNew); // adapt sliding minimum + maximum
	rDynRange.min() = fNew - fRangeLambda * (fNew - rDynRange.min());
	rDynRange.max() = fNew + fRangeLambda * (rDynRange.max() - fNew);

	// compute release mode
	const float fDelta = 0.1 * rAbsRange.range();
	if (fReleased != FLT_MAX && fNew > fCur + fDelta) {
		// if we recently released (fReleased != FLT_MAX) 
		// and fNew increased considerably, we leave release mode
		fReleased = FLT_MAX;
	} else if (fNew < fCur - fDelta) {
		// if new value drops from old value considerably
		// we enter release mode, remembering the old value in fReleased
		fReleased = fCur;
	} else {
		// we are in release mode: linearly decrease fReleased
		fReleased -= fReleaseDecay * rDynRange.range();
		// leave release mode, if we decayed enough
		if (fReleased < rDynRange.min()) {
			fReleased = FLT_MAX;
		}
	}

	// store current value
	fCur = fNew;
}

float TactileSensor::value (Mode mode, float range) const {
	if (mode == rawCurrent) return fCur;
	if (mode == rawMean) return fMean;

	float fRange = rAbsRange.range();
	if (fRange < FLT_EPSILON)
		return 0; // do not divide by zero
	float fMin = rAbsRange.min();
	if (mode == absCurrent) return (fCur - fMin) / fRange;
	if (mode == absMean) return (fMean - fMin) / fRange;

	fRange = rDynRange.range();
	if (fRange < FLT_EPSILON) return 0; // do not divide by zero
	fMin = rDynRange.min();

	// if release mode is active:
	if (mode >= dynCurrentRelease && fReleased != FLT_MAX)
		return - (fReleased - fMin) / fRange;

	switch (mode) {
		case dynCurrent:
		case dynCurrentRelease:
			return (fCur - fMin) / fRange;

		case dynMean:
		case dynMeanRelease:
			return (fMean - fMin) / fRange;

		default: return 0; // should not happen
	}
}

void  TactileSensor::setMeanLambda (float fLambda) {
	this->fMeanLambda=fLambda;
}
void  TactileSensor::setRangeLambda (float fLambda) {
	this->fRangeLambda=fLambda;
}
void  TactileSensor::setReleaseDecay (float fDecay) {
	this->fReleaseDecay=fDecay;
}
