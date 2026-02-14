
#include "Constants.h"
#include "commands/turretyaw/SimpleRotateTurretYaw.h"

#ifndef TESTBOARD

SimpleRotateTurretYaw::SimpleRotateTurretYaw(TurretYawSubsystem *turretYaw, frc::XboxController *operatorController) : m_turretYaw{turretYaw}, m_operatorController{operatorController} {

  AddRequirements(m_turretYaw);
}

void SimpleRotateTurretYaw::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleRotateTurretYaw Initialized\r\n";
#endif
}

void SimpleRotateTurretYaw::Execute() {

  double rotPower = -m_operatorController->GetRawAxis(ControllerConstants::kOperatorLeftXIndex);

  m_turretYaw->SetTurretYawMotorPower(rotPower);
  
}


void SimpleRotateTurretYaw::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleRotateTurretYaw Ended\r\n";
#endif
  m_turretYaw->SetTurretYawMotorPower(0.0);
}
#endif //TESTBOARD