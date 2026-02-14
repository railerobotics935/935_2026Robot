
#include "Constants.h"
#include "commands/turretpitch/SimpleMoveTurretPitch.h"

#ifndef TESTBOARD

SimpleMoveTurretPitch::SimpleMoveTurretPitch(TurretPitchSubsystem *turretPitch, frc::XboxController *operatorController) : m_turretPitch{turretPitch}, m_operatorController{operatorController} {

  AddRequirements(m_turretPitch);
}

void SimpleMoveTurretPitch::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleMoveTurretPitch Initialized\r\n";
#endif
}

void SimpleMoveTurretPitch::Execute() {

  double rotPower;

  if (m_operatorController->GetRawAxis(ControllerConstants::kOperatorLeftTrigger) != 0.0 ) {
    rotPower = -m_operatorController->GetRawAxis(ControllerConstants::kOperatorLeftTrigger);
  }
  
  else if (m_operatorController->GetRawAxis(ControllerConstants::kOperatorRightTrigger) != 0.0 ) {
    rotPower = m_operatorController->GetRawAxis(ControllerConstants::kOperatorRightTrigger);
  }

  else {
    rotPower = 0.0;
  }

  m_turretPitch->SetTurretPitchMotorPower(rotPower);
  
}


void SimpleMoveTurretPitch::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleMoveTurretPitch Ended\r\n";
#endif
  m_turretPitch->SetTurretPitchMotorPower(0.0);
}

#endif //TESTBOARD