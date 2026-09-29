
#include "Constants.h"
#include "commands/stager/ReverseAgitator.h"

#ifndef DISABLEINTAKE

ReverseAgitator::ReverseAgitator(AgitatorSubsystem *agitator) : m_agitator{agitator} {

  AddRequirements(m_agitator);
}

void ReverseAgitator::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "StagerStop Initialized\r\n";
#endif
  m_agitator->SetAgitatorMotorPower(-1.0);
}


void ReverseAgitator::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "StagerStop Ended\r\n";
#endif
  m_agitator->SetAgitatorMotorPower(0.0);
}

#endif //DISABLEINTAKE