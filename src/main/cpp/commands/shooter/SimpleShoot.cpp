
#include "Constants.h"

#ifndef TESTBOARD

#include "commands/shooter/SimpleShoot.h"

SimpleShoot::SimpleShoot(ShooterSubsystem *shooter, frc::XboxController* operatorController) : m_shooter{shooter}, m_operatorController{operatorController} {

  AddRequirements(m_shooter);
}

void SimpleShoot::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleShoot Initialized\r\n";
#endif
}

void SimpleShoot::Execute() {

  double shootPower = m_operatorController->GetRawAxis(ControllerConstants::kOperatorLeftTrigger) * -0.5;

  m_shooter->SetShooterMotorPower(shootPower);
}

void SimpleShoot::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleShoot Ended\r\n";
#endif
  m_shooter->SetShooterMotorPower(0.0);
}

#endif // TESTBOARD