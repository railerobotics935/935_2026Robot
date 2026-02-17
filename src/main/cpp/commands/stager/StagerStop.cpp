
#include "Constants.h"
#include "commands/stager/StagerStop.h"

#ifndef TESTBOARD

StagerStop::StagerStop(StagerSubsystem *stager, frc::XboxController *operatorController) : m_stager{stager}, m_operatorController{operatorController} {

  AddRequirements(m_stager);
}

void StagerStop::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "StagerStop Initialized\r\n";
#endif
  m_stager->SetStagerMotorPower(0.0);
}


void StagerStop::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "StagerStop Ended\r\n";
#endif
  m_stager->SetStagerMotorPower(0.0);
}

#endif //Testboard