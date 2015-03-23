#include "RosAriaNode.h"

int main( int argc, char** argv )
{
  printf("INICIALIZANDO ROS/MAIN");
  ros::init(argc,argv, "RosAria");
  printf("NÓ");
  ros::NodeHandle n;
  printf("INICIALIZANDO ARIA");
  Aria::init();

  printf("INSTANCIANDO");
  RosAriaNode *node = new RosAriaNode(n);

  if( node->Setup() != 0 )
  {
    ROS_FATAL( "RosAria: ROS node setup failed... \n" );
    return -1;
  }
  printf("ANTES DO SPIN");

  node->spin();

  delete node;

  ROS_INFO( "RosAria: Quitting... \n" );
  return 0;
  
}
