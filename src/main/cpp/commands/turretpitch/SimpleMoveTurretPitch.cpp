
#include "Constants.h"
#include "commands/turretpitch/SimpleMoveTurretPitch.h"



SimpleMoveTurretPitch::SimpleMoveTurretPitch(TurretPitchSubsystem *turretPitch, frc::XboxController *operatorController) : m_turretPitch{turretPitch}, m_operatorController{operatorController} {

  AddRequirements(m_turretPitch);
}

void SimpleMoveTurretPitch::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleMoveTurretPitch Initialized\r\n";
#endif
  m_currentTurretPitch = m_turretPitch->GetEncoderValue();
}

void SimpleMoveTurretPitch::Execute() {

  if (m_operatorController->GetRawAxis(ControllerConstants::kOperatorRightYIndex) > 0.0 ) {
    m_currentTurretPitch -= 0.05;
  }
  else if (m_operatorController->GetRawAxis(ControllerConstants::kOperatorRightYIndex) < 0.0) {
    m_currentTurretPitch += 0.05;
  }

  if (m_currentTurretPitch > TurretPitchConstants::kPitchMax) {
    m_currentTurretPitch = TurretPitchConstants::kPitchMax;
  }

  if (m_currentTurretPitch < TurretPitchConstants::kPitchMin) {
    m_currentTurretPitch = TurretPitchConstants::kPitchMin;
  }

  m_turretPitch->SetPitchPosition(m_currentTurretPitch);
  
}


void SimpleMoveTurretPitch::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleMoveTurretPitch Ended\r\n";
#endif
  m_turretPitch->SetTurretPitchMotorPower(0.0);
}
