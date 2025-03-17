package frc.robot.commands;

import frc.robot.StateMachine;
import edu.wpi.first.wpilibj2.command.Command;

public class AlgaeMode extends Command {

    public AlgaeMode() {
        
    }

    @Override
    public void initialize() {
        StateMachine.algaeMode = true;
    }

    @Override
    public void execute() {

    }

    @Override
    public void end(boolean interrupted) {
        StateMachine.algaeMode = false;
    }

    @Override 
    public boolean isFinished(){
        return true;
    }
}
 