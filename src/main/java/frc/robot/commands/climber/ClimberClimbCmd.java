package frc.robot.commands.climber;

import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSys;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberClimbCmd extends Command {

private final ClimberSys climber;   

public ClimberClimbCmd(ClimberSys climber){
    this.climber = climber;

    addRequirements(climber);
}

@Override
public void initialize() {
    climber.setTargetDeg(ClimberConstants.climbingPresetDeg);
}

@Override
public void execute(){

}

@Override
public void end(boolean interrupted) {

}

@Override 
public boolean isFinished(){
    return true;
}
}
 