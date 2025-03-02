package frc.robot.commands.roller;

import frc.robot.Constants.RollerConstants;
import frc.robot.subsystems.RollerSys;
import edu.wpi.first.wpilibj2.command.Command;

public class RollerOuttakeCmd extends Command {

private final RollerSys roller;   

public RollerOuttakeCmd(RollerSys roller){
    this.roller = roller;

    addRequirements(roller);
}

@Override
public void initialize() {
    roller.setPower(RollerConstants.outtakePower);
    // roller.setRPM(RollerConstants.outtakeRPM);
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
 