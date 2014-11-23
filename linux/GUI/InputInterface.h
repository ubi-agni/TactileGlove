#pragma once
#include <boost/function.hpp>

class QString;
class InputInterface {
public:
	typedef boost::function<void(const unsigned short*)> UpdateFunction;
	virtual ~InputInterface() {}

	virtual bool connect(const QString &sInput) = 0;
	virtual bool disconnect() = 0;
	void setUpdateFunction(const UpdateFunction &f) {updateFunc = f;}

protected:
	UpdateFunction updateFunc;
};
