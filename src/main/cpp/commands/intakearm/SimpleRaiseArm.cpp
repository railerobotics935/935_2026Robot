
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

  m_intakeArm->SetArmPosition(IntakeConstants::kIntakeUpperLimit);

}

void SimpleRaiseArm::Execute() {
//  m_currentArmPosition += 0.003;
//
//  if(m_currentArmPosition > IntakeConstants::kIntakeUpperLimit) {
//    m_currentArmPosition = IntakeConstants::kIntakeUpperLimit;
//  }
//
//  m_intakeArm->SetArmPosition(m_currentArmPosition);
}

void SimpleRaiseArm::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleRaiseArm Ended\r\n";
#endif
}

#endif // TESTBOARD