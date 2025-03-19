package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeModeTrue extends Command {

    public AlgaeModeTrue() {
        
    }

    @Override
    public void initialize() {
        RobotContainer.algaeMode = true;
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override 
    public boolean isFinished(){
        return true;
    }
}
 