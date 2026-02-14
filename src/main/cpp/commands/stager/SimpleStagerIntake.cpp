
#include "Constants.h"
#include "commands/stager/SimpleStagerIntake.h"

#ifndef TESTBOARD

SimpleStagerIntake::SimpleStagerIntake(StagerSubsystem *stager) : m_stager{stager} {

  AddRequirements(m_stager);
}

void SimpleStagerIntake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleStagerIntake Initialized\r\n";
#endif
  m_stager->SetStagerMotorPower(-1.0);
}


void SimpleStagerIntake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleStagerIntake Ended\r\n";
#endif
  m_stager->SetStagerMotorPower(0.0);
}

#endif