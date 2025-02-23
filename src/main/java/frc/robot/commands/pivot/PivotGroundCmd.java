package frc.robot.commands.pivot;

import frc.robot.Constants.PivotConstants;
import frc.robot.subsystems.PivotSys;
import edu.wpi.first.wpilibj2.command.Command;

public class PivotGroundCmd extends Command {

private final PivotSys pivot;   

public PivotGroundCmd(PivotSys pivot){
    this.pivot = pivot;

    addRequirements(pivot);
}

@Override
public void initialize() {
    pivot.setTargetDeg(PivotConstants.groundPresetDeg);;
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
 