// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __DIALOGUE_MANAGER1_HPP__
#define __DIALOGUE_MANAGER1_HPP__

#include <yarp/os/all.h>
#include <stdlib.h>

#include "StateMachine.hpp"

//j//#define DEFAULT_FILE_NAME "segRec_ecf_params.xml"

#define VOCAB_FOLLOW_ME VOCAB4('f','o','l','l')
#define VOCAB_STOP_FOLLOWING VOCAB4('s','f','o','l')


namespace teo
{

/**
 * @ingroup dialogueManager1
 *
 * @brief Dialogue Manager 1.
 */
class DialogueManager1 : public yarp::os::RFModule {
  private:
    StateMachine stateMachine;
    yarp::os::BufferedPort<yarp::os::Bottle> inSrPort;
    yarp::os::Port outTtsPort;
    yarp::os::Port outCmdPort;

    bool interruptModule();
    double getPeriod();
    bool updateModule();

  public:
    bool configure(yarp::os::ResourceFinder &rf);
};

}  // namespace teo

#endif  // __DIALOGUE_MANAGER1_HPP__
