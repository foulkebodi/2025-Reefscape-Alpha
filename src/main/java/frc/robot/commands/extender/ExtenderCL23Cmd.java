package frc.robot.commands.extender;

import frc.robot.Constants.ExtenderConstants;
import frc.robot.subsystems.ExtenderSys;
import edu.wpi.first.wpilibj2.command.Command;

public class ExtenderCL23Cmd extends Command {

    private final ExtenderSys extender;   

    public ExtenderCL23Cmd(ExtenderSys extender){
        this.extender = extender;

        addRequirements(extender);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute(){
        extender.setTargetInches(ExtenderConstants.CL23Inches);
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override 
    public boolean isFinished(){
        return true;
    }
}
 