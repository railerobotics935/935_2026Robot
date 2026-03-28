
#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "commands/turretyaw/SimpleTrackingRotate.h"

//#ifndef TESTBOARD

SimpleTrackingRotate::SimpleTrackingRotate(TurretYawSubsystem *turretYaw, DriveSubsystem *drive) : m_turretYaw{turretYaw}, m_drive{drive} {
  AddRequirements(m_turretYaw);
}

void SimpleTrackingRotate::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleTrackingRotate Initialized\r\n";
#endif
}

void SimpleTrackingRotate::Execute() {
  m_turretYaw->UpdateYawAiming(m_drive->GetPose());
}


void SimpleTrackingRotate::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleTrackingRotate Ended\r\n";
#endif
  m_turretYaw->SetTurretYawMotorPower(0.0);
}
//#endif //TESTBOARD