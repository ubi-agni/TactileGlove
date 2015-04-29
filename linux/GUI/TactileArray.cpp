#include "TactileArray.h"
#include <numeric>

TactileArray::TactileArray (size_t n)
	: vSensors(n)
{
}

char TactileArray::getModeID (const std::string& sName) {
	if (sName == "default") return SumPositive;
	else if (sName == "Sum") return Sum;
	else if (sName == "SumPositive") return SumPositive;
	else if (sName == "SumNegative") return SumNegative;
	else if (sName == "CountPositive") return CountPositive;
	else if (sName == "CountNegative") return CountNegative;
	else if (sName == "Min") return Min;
	else if (sName == "Max") return Max;
	return -1;
}
std::string TactileArray::getModeName (AccMode m) {
	switch (m) {
		case Sum: return "Sum";
		case SumPositive: return "SumPositive";
		case SumNegative: return "SumNegative";
		case CountPositive: return "CountPositive";
		case CountNegative: return "CountNegative";
		case Min: return "Min";
		case Max: return "Max";
		default: return "";
	}
}

TactileArray::vector_data TactileArray::getAbsValues () const {
	vector_data vReturn(vSensors.size());
	vector_data::iterator v = vReturn.begin();
	for (const_iterator it=begin(), e=end(); it!=e; ++it, ++v)
		*v = it->value(TactileSensor::rawCurrent);
	return vReturn;
}
TactileArray::vector_data 
TactileArray::getValues (TactileSensor::Mode mode,
								 float fMinRange) const {
	vector_data vReturn(vSensors.size());
	copyValues(vReturn.begin(), mode, fMinRange);
	return vReturn;
}

static float posAdd (float result, float c) {
	if (c > 0) return result+c;
	return result;
}
static float negAdd (float result, float c) {
	if (c < 0) return result+c;
	return result;
}
static float posCount (float result, float c) {
	if (c > 0) return result+1;
	return result;
}
static float negCount (float result, float c) {
	if (c < 0) return result+1;
	return result;
}
static float minFunc (float a, float b) {return std::min(a,b);}
static float maxFunc (float a, float b) {return std::max(a,b);}
float TactileArray::accumulate (const TactileArray::vector_data& data, 
                                AccMode mode, bool bMean) {
	float fRet=0;
	switch (mode) {
		case Sum: 
			fRet = std::accumulate(data.begin(),data.end(), 0.);
			break;

		case SumPositive:
			fRet = std::accumulate(data.begin(),data.end(), 0., posAdd);
			break;
		case SumNegative:
			fRet = std::accumulate(data.begin(),data.end(), 0., negAdd);
			break;

		case CountPositive:
			fRet = std::accumulate(data.begin(),data.end(), 0., posCount);
			break;
		case CountNegative:
			fRet = std::accumulate(data.begin(),data.end(), 0., negCount);
			break;

		case Min:
			fRet = std::accumulate(data.begin(),data.end(), FLT_MAX, minFunc);
			break;
		case Max:
			fRet = std::accumulate(data.begin(),data.end(), -FLT_MAX, maxFunc);
			break;

		case lastMode:
			break;
	}
	if (bMean) return fRet / data.size();
	else return fRet;
}

void TactileArray::setMeanLambda (double fLambda) {
	for (iterator it=begin(), e=end(); it!=e; ++it)
		it->setMeanLambda(fLambda);
}
void TactileArray::setRangeLambda (double fLambda) {
	for (iterator it=begin(), e=end(); it!=e; ++it)
		it->setRangeLambda(fLambda);
}
void TactileArray::setReleaseDecay (double fDecay) {
	for (iterator it=begin(), e=end(); it!=e; ++it)
		it->setReleaseDecay(fDecay);
}

