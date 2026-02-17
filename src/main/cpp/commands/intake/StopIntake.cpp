
#include "Constants.h"

#ifndef TESTBOARD

#include "commands/intake/StopIntake.h"
StopIntake::StopIntake(IntakeSubsystem *intake, frc::XboxController* operatorController) : m_intake{intake}, m_operatorController{operatorController} {
    AddRequirements(m_intake);
}
void StopIntake::Initialize() {
  #ifdef PRINTDEBUG
    std::cout << "StopIntake Initialized\r\n";
  #endif
    m_intake->SetIntakeMotorPower(0.0);
}
void StopIntake::End(bool interrupted) {
  #ifdef PRINTDEBUG
    std::cout << "StopIntake Ended\r\n";
  #endif
    m_intake->SetIntakeMotorPower(0.0);
}
#endif //TESTBOARD