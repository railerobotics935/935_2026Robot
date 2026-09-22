
#include "Constants.h"
#include "commands/stager/AgitatorStop.h"

#ifndef DISABLEINTAKE

AgitatorStop::AgitatorStop(AgitatorSubsystem *agitator, frc::XboxController* controller) : m_agitator{agitator}, m_controller{controller} {

  AddRequirements(m_agitator);
}

void AgitatorStop::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "StagerStop Initialized\r\n";
#endif
  m_agitator->SetAgitatorMotorPower(0.0);
}


void AgitatorStop::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "StagerStop Ended\r\n";
#endif
  m_agitator->SetAgitatorMotorPower(0.0);
}

#endif //DISABLEINTAKE