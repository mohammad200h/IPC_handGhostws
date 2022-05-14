
#include "GhostInterface.h"


#include <vector>
using namespace std;
class GhostInterface_G* GhostExampleCreateFunc(struct CommonExampleOptions_G& options,int num_env);

vector<double> getEuler(btQuaternion q);
btQuaternion getQuaternionFromEuler(float rollX, float pitchY, float yawZ);



