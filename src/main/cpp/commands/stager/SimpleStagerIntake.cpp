
#include "Constants.h"
#include "commands/stager/SimpleStagerIntake.h"

#ifndef TESTBOARD

SimpleStagerIntake::SimpleStagerIntake(StagerSubsystem *stager, AgitatorSubsystem* agitator, frc::XboxController* operatorController) : m_stager{stager}, m_agitator{agitator}, m_operatorController{operatorController} {

  AddRequirements(m_stager);
  AddRequirements(m_agitator);
}

void SimpleStagerIntake::Initialize() {
#ifdef PRINTDEBUG
  std::cout << "SimpleStagerIntake Initialized\r\n";
#endif

}

void SimpleStagerIntake::Execute() {

  double stagePower;
  if(m_operatorController->GetRawAxis(ControllerConstants::kOperatorRightTrigger) != 0.0) {
    stagePower = 1.0;
  }
  else {
    stagePower = 0.0;
  }
  m_stager->SetStagerMotorPower(stagePower);
  m_agitator->SetAgitatorMotorPower(stagePower);

}

void SimpleStagerIntake::End(bool interrupted) {
#ifdef PRINTDEBUG
  std::cout << "SimpleStagerIntake Ended\r\n";
#endif
  m_stager->SetStagerMotorPower(0.0);
  m_agitator->SetAgitatorMotorPower(0.0);
}

#endif