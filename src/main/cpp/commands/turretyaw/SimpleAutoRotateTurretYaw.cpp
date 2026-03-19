#if 0
#include "Constants.h"
#include "commands/turretyaw/SimpleAutoRotateTurretYaw.h"

//#ifndef TESTBOARD

SimpleAutoRotateTurretYaw::SimpleAutoRotateTurretYaw(TurretYawSubsystem *turretYaw, frc::XboxController *operatorController) : m_turretYaw{turretYaw}, m_operatorController{operatorController} {

  AddRequirements(m_turretYaw);
}

void SimpleAutoRotateTurretYaw::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleAutoRotateTurretYaw Initialized\r\n";
#endif
}

void SimpleAutoRotateTurretYaw::Execute() {
  m_turretYaw->GetCurrentYaw();
}


void SimpleAutoRotateTurretYaw::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleAutoRotateTurretYaw Ended\r\n";
#endif
  m_turretYaw->SetTurretYawMotorPower(0.0);
}
#endif