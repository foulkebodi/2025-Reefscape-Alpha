package frc.robot.commands;

import frc.robot.Constants.State;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj2.command.Command;

public class StateHOME extends Command {

    public StateHOME() {
        
    }

    @Override
    public void initialize() {
        RobotContainer.currentState = State.HOME;
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
 