// #include <alcommon/albroker.h>
// #include <alcommon/albrokermanager.h>
// #include <alcommon/almodule.h>
// #include <alcommon/altoolsmain.h>
// #include <signal.h>
// #include <boost/shared_ptr.hpp>
// #include "mc_naoqi_dcm.h"

// #ifdef _WIN32
// #define ALCALL __declspec(dllexport)
// #else
// #define ALCALL
// #endif

// extern "C" {
// ALCALL int _createModule(boost::shared_ptr<AL::ALBroker> pBroker)
// {
//   // init broker with the main broker instance
//   // from the parent executable
//   AL::ALBrokerManager::setInstance(pBroker->fBrokerManager.lock());
//   AL::ALBrokerManager::getInstance()->addBroker(pBroker);

//   // create module instances
//   AL::ALModule::createModule<mc_naoqi_dcm::MCNAOqiDCM>(pBroker, "MCNAOqiDCM");
//   return 0;
// }

// ALCALL int _closeModule()
// {
//   return 0;
// }

// }  // extern "C"


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