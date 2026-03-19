
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

  //m_intakeArm->SetArmPosition(IntakeConstants::kIntakeUpperLimit);
    m_currentIntakeArmHeight = m_intakeArm->GetEncoderValue();


}

void SimpleRaiseArm::Execute() {
 m_currentIntakeArmHeight += 0.003;
//
//  if(m_currentArmPosition > IntakeConstants::kIntakeUpperLimit) {
//    m_currentArmPosition = IntakeConstants::kIntakeUpperLimit;
//  }
//
//  m_intakeArm->SetArmPosition(m_currentArmPosition);
  if (m_currentIntakeArmHeight > IntakeConstants::kIntakeUpperLimit) {
    m_currentIntakeArmHeight = IntakeConstants::kIntakeUpperLimit;
  }

  m_intakeArm->SetArmPosition(m_currentIntakeArmHeight);
}

void SimpleRaiseArm::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleRaiseArm Ended\r\n";
#endif
m_intakeArm->SetIntakeArmMotorPower(0.1);

#endif // TESTBOARD
}