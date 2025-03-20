package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.Constants.State;
import edu.wpi.first.wpilibj2.command.Command;

public class StateCL4 extends Command {

    public StateCL4() {
        
    }

    @Override
    public void initialize() {
        RobotContainer.currentState = State.CL4;
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
 