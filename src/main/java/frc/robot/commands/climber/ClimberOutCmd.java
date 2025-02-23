package frc.robot.commands.climber;

import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSys;
import edu.wpi.first.wpilibj2.command.Command;

public class ClimberOutCmd extends Command {

private final ClimberSys climber;   

public ClimberOutCmd(ClimberSys climber){
    this.climber = climber;

    addRequirements(climber);
}

@Override
public void initialize() {
    climber.setTargetDeg(ClimberConstants.outPresetDeg);
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
 