
#include "Constants.h"

#ifndef TESTBOARD

#include "commands/intakearm/SimpleRaiseArm.h"

SimpleRaiseArm::SimpleRaiseArm(IntakeArmSubsystem *intakeArm) : m_intakeArm{intakeArm} {

  AddRequirements(m_intakeArm);
}

void SimpleRaiseArm::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleRaiseArm Initialized\r\n";
#endif
  m_intakeArm->SetIntakeArmMotorPower(-0.3);
}


void SimpleRaiseArm::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleRaiseArm Ended\r\n";
#endif
  m_intakeArm->SetIntakeArmMotorPower(0.0);
}

#endif // TESTBOARD