package frc.robot.commands;

import frc.robot.Constants.State;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class StateCL1 extends Command {

    public StateCL1() {
        
    }

    @Override
    public void initialize() {
        RobotContainer.currentState = State.CL1;
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
 