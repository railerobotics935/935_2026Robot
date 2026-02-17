
#include "Constants.h"

#ifndef TESTBOARD

#include "commands/shooter/StopShooter.h"
StopShooter::StopShooter(ShooterSubsystem *shooter, frc::XboxController* operatorController) : m_shooter{shooter}, m_operatorController{operatorController} {
    AddRequirements(m_shooter);
}
void StopShooter::Initialize() {
  #ifdef PRINTDEBUG
    std::cout << "StopShooter Initialized\r\n";
  #endif
    m_shooter->SetShooterMotorPower(0.0);
}
void StopShooter::End(bool interrupted) {
  #ifdef PRINTDEBUG
    std::cout << "StopShooter Ended\r\n";
  #endif
    m_shooter->SetShooterMotorPower(0.0);
}
#endif //TESTBOARD