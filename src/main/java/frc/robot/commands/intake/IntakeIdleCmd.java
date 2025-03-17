package frc.robot.commands.intake;

import frc.robot.subsystems.IntakeSys;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeIdleCmd extends Command {

    private final IntakeSys intake;   

    public IntakeIdleCmd(IntakeSys intake){
        this.intake = intake;

        addRequirements(intake);
    }

    @Override
    public void initialize() {
        intake.setIsIntaking(false);
        intake.setIsOuttaking(false);
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
 