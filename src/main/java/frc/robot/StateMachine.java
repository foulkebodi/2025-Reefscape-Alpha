package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.State;

public class StateMachine {
    public SequentialCommandGroup getSequence(State currentState, State targetState) {
        if(currentState == State.INTAKING && targetState == State.AH) {
            return new SequentialCommandGroup(null);
        } else {
            return null;
        }
    }
}
