
#include "Constants.h"

#ifndef TESTBOARD

#include "commands/intakearm/SimpleLowerArm.h"

SimpleLowerArm::SimpleLowerArm(IntakeArmSubsystem *intakeArm) : m_intakeArm{intakeArm} {

  AddRequirements(m_intakeArm);
}

void SimpleLowerArm::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleLowerArm Initialized\r\n";
#endif
  m_intakeArm->SetIntakeArmMotorPower(0.3);
}


void SimpleLowerArm::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleLowerArm Ended\r\n";
#endif
  m_intakeArm->SetIntakeArmMotorPower(0.0);
}

#endif // TESTBOARD