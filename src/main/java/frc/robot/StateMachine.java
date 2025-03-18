package frc.robot;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.State;
import frc.robot.commands.transitions.AL2ToAlgaeHome;
import frc.robot.commands.transitions.AL3ToAlgaeHome;
import frc.robot.commands.transitions.AlgaeHomeToAL2;
import frc.robot.commands.transitions.AlgaeHomeToAL3;
import frc.robot.commands.transitions.AlgaeHomeToBarge;
import frc.robot.commands.transitions.AlgaeHomeToClimb;
import frc.robot.commands.transitions.AlgaeHomeToCoralHome;
import frc.robot.commands.transitions.AlgaeHomeToGround;
import frc.robot.commands.transitions.AlgaeHomeToProcessor;
import frc.robot.commands.transitions.BargeToAlgaeHome;
import frc.robot.commands.transitions.CL1ToCoralHome;
import frc.robot.commands.transitions.CL23ToCoralHome;
import frc.robot.commands.transitions.CL4ToCoralHome;
import frc.robot.commands.transitions.ChuteToCoralHome;
import frc.robot.commands.transitions.ClimbToAlgaeHome;
import frc.robot.commands.transitions.ClimbToCoralHome;
import frc.robot.commands.transitions.CoralHomeToAlgaeHome;
import frc.robot.commands.transitions.CoralHomeToCL1;
import frc.robot.commands.transitions.CoralHomeToCL2;
import frc.robot.commands.transitions.CoralHomeToCL3;
import frc.robot.commands.transitions.CoralHomeToCL4;
import frc.robot.commands.transitions.CoralHomeToChute;
import frc.robot.commands.transitions.CoralHomeToClimb;
import frc.robot.commands.transitions.GroundToAlgaeHome;
import frc.robot.commands.transitions.ProcessorToAlgaeHome;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;

public class StateMachine {
    public static State currentState = State.CHUTE;
    public static boolean algaeMode = false;
    private final PivotSys pivot = new PivotSys();
    private final ElevatorSys elevator = new ElevatorSys();
    private final ExtenderSys extender = new ExtenderSys();


    public SequentialCommandGroup getSequence(State targetState) {
        if(currentState == State.HOME && targetState == State.HOME && !algaeMode) {
            currentState = State.HOME;
            return new AlgaeHomeToCoralHome(pivot, elevator, extender); // algae home to coral home
        } else if (currentState == State.HOME && targetState == State.HOME && algaeMode) {
            currentState = State.HOME;
            return new CoralHomeToAlgaeHome(pivot, elevator, extender); // coral home to algae home
        } else if (currentState == State.HOME && targetState == State.CHUTE && !algaeMode) {
            currentState = State.HOME;
            return new CoralHomeToChute(pivot, elevator, extender); // coral home to chute
        } else if (currentState == State.HOME && targetState == State.CL1 && !algaeMode) {
            currentState = State.CL1;
            return new CoralHomeToCL1(pivot, elevator, extender); // coral home to cl1
        } else if (currentState == State.HOME && targetState == State.CL2 && !algaeMode) {
            currentState = State.CL2;
            return new CoralHomeToCL2(pivot, elevator, extender); // coral home to cl2
        } else if (currentState == State.HOME && targetState == State.CL3 && !algaeMode) {
            currentState = State.CL3;
            return new CoralHomeToCL3(pivot, elevator, extender); // coral home to cl3
        } else if (currentState == State.HOME && targetState == State.CL4 && !algaeMode) {
            currentState = State.CL4;
            return new CoralHomeToCL4(pivot, elevator, extender); // coral home to cl4
        } else if (currentState == State.HOME && targetState == State.CL2 && algaeMode) {
            currentState = State.CL2;
            return new AlgaeHomeToAL2(pivot, elevator, extender); // algae home to al2
        } else if (currentState == State.HOME && targetState == State.CL3 && algaeMode) {
            currentState = State.CL3;
            return new AlgaeHomeToAL3(pivot, elevator, extender); // algae home to al3
        } else if (currentState == State.HOME && targetState == State.CL4 && algaeMode) {
            currentState = State.CL4;
            return new AlgaeHomeToBarge(pivot, elevator, extender); // algae home to barge
        } else if (currentState == State.HOME && targetState == State.PROCESSOR && algaeMode) {
            currentState = State.PROCESSOR;
            return new AlgaeHomeToProcessor(pivot, elevator, extender); // algae home to processor
        } else if (currentState == State.HOME && targetState == State.GROUND && algaeMode) {
            currentState = State.GROUND;
            return new AlgaeHomeToGround(pivot, elevator, extender); // algae home to ground
        } else if (currentState == State.CL1 && targetState == State.HOME && !algaeMode) {
            currentState = State.HOME;
            return new CL1ToCoralHome(pivot, elevator, extender); // cl1 to  coral home
        } else if (currentState == State.CL2 && targetState == State.HOME && !algaeMode) {
            currentState = State.HOME;
            return new CL23ToCoralHome(pivot, elevator, extender); // cl2 to coral home
        } else if (currentState == State.CL3 && targetState == State.HOME && !algaeMode) {
            currentState = State.HOME;
            return new CL23ToCoralHome(pivot, elevator, extender); // cl3 to coral home
        } else if (currentState == State.CL4 && targetState == State.HOME && !algaeMode) {
            currentState = State.HOME; 
            return new CL4ToCoralHome(pivot, elevator, extender);// cl4 to coral home
        } else if (currentState == State.GROUND && targetState == State.HOME && algaeMode) {
            currentState = State.HOME;
            return new GroundToAlgaeHome(pivot, elevator, extender); // ground to algae home
        } else if (currentState == State.PROCESSOR && targetState == State.HOME && algaeMode) {
            currentState = State.HOME;
            return new ProcessorToAlgaeHome(pivot, elevator, extender); // processor to algae home
        } else if (currentState == State.CL4 && targetState == State.HOME && algaeMode) {
            currentState = State.HOME;
            return new BargeToAlgaeHome(pivot, elevator, extender); // barge to algae home
        } else if (currentState == State.CL2 && targetState == State.HOME && algaeMode) {
            currentState = State.HOME;
            return new AL2ToAlgaeHome(pivot, elevator, extender); // al2 to algae home
        } else if (currentState == State.CL3 && targetState == State.HOME && algaeMode) {
            currentState = State.HOME;
            return new AL3ToAlgaeHome(pivot, elevator, extender); // al3 to algae home
        } else if (currentState == State.CHUTE && targetState == State.HOME && !algaeMode) {
            currentState = State.HOME;
            return new ChuteToCoralHome(pivot, elevator, extender); // chute to coral home
        } else if (currentState == State.HOME && targetState == State.CLIMB && algaeMode) {
            currentState = State.HOME;
            return new AlgaeHomeToClimb(pivot, elevator, extender); // algae home to climb
        } else if (currentState == State.HOME && targetState == State.CLIMB && !algaeMode) {
            currentState = State.HOME;
            return new CoralHomeToClimb(pivot, elevator, extender); // coral home to climb
        } else if (currentState == State.CLIMB && targetState == State.HOME && !algaeMode) {
            currentState = State.HOME;
            return new ClimbToCoralHome(pivot, elevator, extender); // climb to coral home
        } else if (currentState == State.CLIMB && targetState == State.HOME && algaeMode) {
            currentState = State.HOME;
            return new ClimbToAlgaeHome(pivot, elevator, extender); // climb to algae home
        } else {
            return null;
        }
    }

    public String getCurrentStateAsString() {
        if(currentState == State.HOME && algaeMode) {
            return "AH";
        } else if(currentState == State.AL2) {
            return "AL2";
        } else if(currentState == State.AL3) {
            return "AL3";
        } else if(currentState == State.BARGE) {
            return "BARGE";
        } else if(currentState == State.HOME && !algaeMode) {
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
        } else if(currentState == State.CHUTE) {
            return "INTAKING";
        } else if(currentState == State.PROCESSOR) {
            return "PROCESSOR";
        } else {
            return "null";
        }
    }
}
