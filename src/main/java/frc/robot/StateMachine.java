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

    public String getCurrentStateAsString() {
        if(currentState == State.AH) {
            return "AH";
        } else if(currentState == State.AL2) {
            return "AL2";
        } else if(currentState == State.AL3) {
            return "AL3";
        } else if(currentState == State.BARGE) {
            return "BARGE";
        } else if(currentState == State.CH) {
            return "CH";
        } else if(currentState == State.CL1) {
            return "CL1";
        } else if(currentState == State.CL2) {
            return "CL2";
        } else if(currentState == State.CL3) {
            return "CL3";
        } else if(currentState == State.CL4) {
            return "CL4";
        } else if(currentState == State.GROUND) {
            return "GROUND";
        } else if(currentState == State.INTAKING) {
            return "INTAKING";
        } else if(currentState == State.PROCESSOR) {
            return "PROCESSOR";
        } else {
            return "null";
        }
    }
}
