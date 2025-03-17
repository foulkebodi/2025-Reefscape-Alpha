package frc.robot.commands.intake;

import frc.robot.subsystems.IntakeSys;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeOuttakeCmd extends Command {

private final IntakeSys Intake;   

public IntakeOuttakeCmd(IntakeSys Intake){
    this.Intake = Intake;

    addRequirements(Intake);
}

@Override
public void initialize() {
    Intake.setIsOuttaking(true);
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
 