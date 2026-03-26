
#include "commands/autocommands/SimpleAutoAgitate.h"


SimpleAutoAgitate::SimpleAutoAgitate(AgitatorSubsystem* agitator) : m_agitator{agitator} {
  // Initilize local copys of pointers

  // Add reqierments for the command
  AddRequirements(m_agitator);
}

void SimpleAutoAgitate::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleAutoAgitate Initialized\r\n";
#endif
  m_agitator->SetAgitatorMotorPower(0.5);
  
}

void SimpleAutoAgitate::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleAutoAgitate Ended\r\n";
#endif
  m_agitator->SetAgitatorMotorPower(0.0);

}