package frc.robot.commands.winch;

import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.WinchSys;
import edu.wpi.first.wpilibj2.command.Command;

public class WinchCmd extends Command {

private final WinchSys winch;   

public WinchCmd(WinchSys winch){
    this.winch = winch;

    addRequirements(winch);
}

@Override
public void initialize() {
    winch.setWinchTargetDeg(ClimberConstants.winchPresetInches);
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
 