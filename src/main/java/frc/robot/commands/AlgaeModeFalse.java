package frc.robot.commands;

import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeModeFalse extends Command {

    public AlgaeModeFalse() {
        
    }

    @Override
    public void initialize() {
        RobotContainer.algaeMode = false;
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
 