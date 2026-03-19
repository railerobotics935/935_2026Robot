
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

 m_currentArmPosition = m_intakeArm->GetEncoderValue();
 


}

void SimpleLowerArm::Execute() {
  m_currentArmPosition -= 0.003;

  if(m_currentArmPosition < IntakeConstants::kIntakeLowerLimit) {
    m_currentArmPosition = IntakeConstants::kIntakeLowerLimit;
  }

 m_intakeArm->SetArmPosition(m_currentArmPosition);
}

void SimpleLowerArm::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleLowerArm Ended\r\n";
#endif
}

#endif // TESTBOARD