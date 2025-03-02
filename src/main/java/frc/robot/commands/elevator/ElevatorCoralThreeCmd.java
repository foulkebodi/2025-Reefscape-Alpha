package frc.robot.commands.elevator;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSys;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorCoralThreeCmd extends Command {

private final ElevatorSys elevator;   

public ElevatorCoralThreeCmd(ElevatorSys elevator){
    this.elevator = elevator;

    addRequirements(elevator);
}

@Override
public void initialize() {
    elevator.setTargetInches(ElevatorConstants.coralThreePresetInches);
}
@Override
public void execute(){
}
@Override
public void end(boolean interrupted) {}

@Override 
public boolean isFinished(){
    return true;
}
}
 