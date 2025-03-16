package frc.robot.commands.roller;

import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.IntakeSys;
import edu.wpi.first.wpilibj2.command.Command;

public class RollerIdleCmd extends Command {

private final IntakeSys intake;   

public RollerIdleCmd(IntakeSys intake){
    this.intake = intake;

    addRequirements(intake);
}

@Override
public void initialize() {
    intake.setPower(RollerConstants.idlePower);
    // roller.setRPM(RollerConstants.idleRPM);
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
 