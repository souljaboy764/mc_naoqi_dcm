#include <iostream>
#include <string>
#include <qi/session.hpp>
#include <qi/applicationsession.hpp>
#include <boost/shared_ptr.hpp>
#include "mc_naoqi_dcm.h"

int main(int argc, char* argv[])
{
  qi::ApplicationSession app(argc, argv);
  app.start();
  qi::SessionPtr session = app.session();
  session->registerService("MCNAOqiDCM", qi::AnyObject(boost::make_shared<mc_naoqi_dcm::MCNAOqiDCM>(session)));
  app.run();
  
  return 0;
}