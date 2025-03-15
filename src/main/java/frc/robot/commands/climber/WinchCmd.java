package frc.robot.commands.climber;

import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSys;
import edu.wpi.first.wpilibj2.command.Command;

public class WinchCmd extends Command {

private final ClimberSys climber;   

public WinchCmd(ClimberSys climber){
    this.climber = climber;

    addRequirements(climber);
}

@Override
public void initialize() {
    climber.setWinchTargetDeg(ClimberConstants.winchPresetInches);
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
 