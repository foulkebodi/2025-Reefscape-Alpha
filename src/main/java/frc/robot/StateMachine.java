package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.State;
import frc.robot.commands.transitions.IntakingToCoralHome;
import frc.robot.subsystems.PivotSys;

public class StateMachine {
    public static State currentState = State.INTAKING;
    public static boolean algaeMode = false;
    private final PivotSys pivot = new PivotSys();

    public SequentialCommandGroup getSequence(State targetState) {
        if(currentState == State.INTAKING && targetState == State.CH) {
            currentState = State.AH;
            return new IntakingToCoralHome(pivot);
        } else {
            return null;
        }
    }
}
