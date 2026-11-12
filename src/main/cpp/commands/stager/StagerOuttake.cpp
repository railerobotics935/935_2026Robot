#include "Constants.h"
#include "commands/stager/StagerOuttake.h"

#ifndef DISABLEINTAKE

StagerOuttake::StagerOuttake(StagerSubsystem *stager) : m_stager{stager} {

  AddRequirements(m_stager);
}

void StagerOuttake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "StagerOuttake Initialized\r\n";
#endif
  m_stager->SetStagerMotorPower(1.0);
}


void StagerOuttake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "StagerOuttake Ended\r\n";
#endif
  m_stager->SetStagerMotorPower(0.0);
}

#endif //DISABLEINTAKE