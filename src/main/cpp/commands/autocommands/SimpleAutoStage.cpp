
#include "commands/autocommands/SimpleAutoStage.h"


SimpleAutoStage::SimpleAutoStage(StagerSubsystem* stager) : m_stager{stager} {
  // Initilize local copys of pointers

  // Add reqierments for the command
  AddRequirements(m_stager);
}

void SimpleAutoStage::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleAutoStage Initialized\r\n";
#endif
  m_stager->SetStagerMotorPower(0.5);
  
}

void SimpleAutoStage::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleAutoStage Ended\r\n";
#endif
  m_stager->SetStagerMotorPower(0.0);
  //m_coralShoot->SetCoralShootAngle(0.0);

}