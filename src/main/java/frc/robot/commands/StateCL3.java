package frc.robot.commands;

import frc.robot.RobotContainer;
import frc.robot.Constants.State;
import edu.wpi.first.wpilibj2.command.Command;

public class StateCL3 extends Command {

    public StateCL3() {
        
    }

    @Override
    public void initialize() {
        RobotContainer.currentState = State.CL3;
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
 