
#include "GhostWorld.h"

using namespace std;

// https://discourse.panda3d.org/
// https://discourse.panda3d.org/c/c-coding-using-panda3d/7

int main(int argc, char* argv[])
{
 
  vector<float> offset_size = {1,1,1};
  int num_env =1;
  GhostWorld GW = GhostWorld(num_env,offset_size,argc, argv);
  GW.setup_camera();
  GW.run();
  GW.closeFramework();
      

 
  return 0;
}