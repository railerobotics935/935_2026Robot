
#include "commands/autocommands/SimpleAutoShoot.h"


SimpleAutoShoot::SimpleAutoShoot(ShooterSubsystem* shooter) : m_shooter{shooter} {
  // Initilize local copys of pointers

  // Add reqierments for the command
  AddRequirements(m_shooter);
}

void SimpleAutoShoot::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleAutoShoot Initialized\r\n";
#endif
  m_shooter->SetShooterMotorPower(-0.5);
  
}

void SimpleAutoShoot::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleAutoShoot Ended\r\n";
#endif
  m_shooter->SetShooterMotorPower(0.0);
  //m_coralShoot->SetCoralShootAngle(0.0);

}