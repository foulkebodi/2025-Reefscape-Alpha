package frc.robot.commands.climber;

import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSys;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberHomeCmd extends Command {

private final ClimberSys climber;   

public ClimberHomeCmd(ClimberSys climber){
    this.climber = climber;

    addRequirements(climber);
}

@Override
public void initialize() {
    climber.setClimberTargetDeg(ClimberConstants.homePresetDeg);
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
 