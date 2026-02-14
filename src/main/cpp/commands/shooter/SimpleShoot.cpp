
#include "Constants.h"

#ifndef TESTBOARD

#include "commands/shooter/SimpleShoot.h"

SimpleShoot::SimpleShoot(ShooterSubsystem *shooter) : m_shooter{shooter} {

  AddRequirements(m_shooter);
}

void SimpleShoot::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleShoot Initialized\r\n";
#endif
  m_shooter->SetShooterMotorPower(-0.5);
}


void SimpleShoot::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleShoot Ended\r\n";
#endif
  m_shooter->SetShooterMotorPower(0.0);
}

#endif // TESTBOARD