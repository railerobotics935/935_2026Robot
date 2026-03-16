
#include "Constants.h"

#ifndef TESTBOARD

#include "commands/intakearm/SimpleStopArm.h"

SimpleStopArm::SimpleStopArm(IntakeArmSubsystem *intakeArm) : m_intakeArm{intakeArm} {

  AddRequirements(m_intakeArm);
}

void SimpleStopArm::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleLowerArm Initialized\r\n";
#endif
  m_intakeArm->SetIntakeArmMotorPower(0.0);
}


void SimpleStopArm::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleLowerArm Ended\r\n";
#endif
  m_intakeArm->SetIntakeArmMotorPower(0.0);
}

#endif // TESTBOARD