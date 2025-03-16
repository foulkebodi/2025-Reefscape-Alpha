package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.State;

public class StateMachine {
    public static State currentState = State.INTAKING;
    public static boolean algaeMode = false;

    public SequentialCommandGroup getSequence(State targetState) {
        if(currentState == State.INTAKING && targetState == State.AH) {
            currentState = State.AH;
            return new SequentialCommandGroup(null);
        } else {
            return null;
        }
    }
}
