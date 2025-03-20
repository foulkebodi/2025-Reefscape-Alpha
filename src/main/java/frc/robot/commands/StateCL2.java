package frc.robot.commands;

import frc.robot.Constants.State;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class StateCL2 extends Command {

    public StateCL2() {
        
    }

    @Override
    public void initialize() {
        RobotContainer.currentState = State.CL2;
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
 