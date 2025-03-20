package frc.robot.commands;

import frc.robot.Constants.State;
import frc.robot.RobotContainer;
import frc.robot.commands.transitions.AL2ToHome;
import frc.robot.commands.transitions.AL3ToHome;
import frc.robot.commands.transitions.BargeToHome;
import frc.robot.commands.transitions.CL1ToHome;
import frc.robot.commands.transitions.CL2ToHome;
import frc.robot.commands.transitions.CL3ToHome;
import frc.robot.commands.transitions.CL4ToHome;
import frc.robot.commands.transitions.ChuteToHome;
import frc.robot.commands.transitions.ClimbToHome;
import frc.robot.commands.transitions.GroundToHome;
import frc.robot.commands.transitions.HomeToAL2;
import frc.robot.commands.transitions.HomeToAL3;
import frc.robot.commands.transitions.HomeToBarge;
import frc.robot.commands.transitions.HomeToCL1;
import frc.robot.commands.transitions.HomeToCL2;
import frc.robot.commands.transitions.HomeToCL3;
import frc.robot.commands.transitions.HomeToCL4;
import frc.robot.commands.transitions.HomeToChute;
import frc.robot.commands.transitions.HomeToClimb;
import frc.robot.commands.transitions.HomeToGround;
import frc.robot.commands.transitions.HomeToProcessor;
import frc.robot.commands.transitions.ProcessorToHome;
import frc.robot.commands.util.DoNothings;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;
import frc.robot.subsystems.WinchSys;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class GetSequenceCmd extends Command {
    private final PivotSys pivotSys;
    private final ElevatorSys elevatorSys;
    private final ExtenderSys extenderSys;
    private final WinchSys winchSys;
    private final State targetState;

    public GetSequenceCmd(PivotSys pivotSys, ElevatorSys elevatorSys, ExtenderSys extenderSys, State targetState, WinchSys winchSys) {
        this.pivotSys = pivotSys;
        this.elevatorSys = elevatorSys;
        this.extenderSys = extenderSys;
        this.targetState = targetState;
        this.winchSys = winchSys;
    }

    @Override
    public void initialize() {
        if(RobotContainer.currentState == State.HOME && targetState == State.CHUTE) {
            RobotContainer.currentState = State.CHUTE;
            CommandScheduler.getInstance().schedule(new HomeToChute(pivotSys, elevatorSys, extenderSys)); // home to chute
        }  else if (RobotContainer.currentState == State.CHUTE && targetState == State.HOME) {
            RobotContainer.currentState = State.HOME;
            CommandScheduler.getInstance().schedule(new ChuteToHome(pivotSys, elevatorSys, extenderSys)); // chute to home
        } else if (RobotContainer.currentState == State.HOME && targetState == State.CL1 && !RobotContainer.algaeMode) {
            RobotContainer.currentState = State.CL1;
            CommandScheduler.getInstance().schedule(new HomeToCL1(pivotSys, elevatorSys, extenderSys)); // home to cl1
        } else if (RobotContainer.currentState == State.HOME && targetState == State.CL2 && !RobotContainer.algaeMode) {
            RobotContainer.currentState = State.CL2;
            CommandScheduler.getInstance().schedule(new HomeToCL2(pivotSys, elevatorSys, extenderSys)); // home to cl2
        } else if (RobotContainer.currentState == State.HOME && targetState == State.CL3 && !RobotContainer.algaeMode) {
            RobotContainer.currentState = State.CL3;
            CommandScheduler.getInstance().schedule(new HomeToCL3(pivotSys, elevatorSys, extenderSys)); // home to cl3
        } else if (RobotContainer.currentState == State.HOME && targetState == State.CL4 && !RobotContainer.algaeMode) {
            RobotContainer.currentState = State.CL4;
            CommandScheduler.getInstance().schedule(new HomeToCL4(pivotSys, elevatorSys, extenderSys)); // home to cl4
        } else if (RobotContainer.currentState == State.HOME && targetState == State.CL2 && RobotContainer.algaeMode) {
            RobotContainer.currentState = State.CL2;
            CommandScheduler.getInstance().schedule(new HomeToAL2(pivotSys, elevatorSys, extenderSys)); // home to al2
        } else if (RobotContainer.currentState == State.HOME && targetState == State.CL3 && RobotContainer.algaeMode) {
            RobotContainer.currentState = State.CL3;
            CommandScheduler.getInstance().schedule(new HomeToAL3(pivotSys, elevatorSys, extenderSys)); // home to al3
        } else if (RobotContainer.currentState == State.HOME && targetState == State.CL4 && RobotContainer.algaeMode) {
            RobotContainer.currentState = State.CL4;
            CommandScheduler.getInstance().schedule(new HomeToBarge(pivotSys, elevatorSys, extenderSys)); // home to barge
        } else if (RobotContainer.currentState == State.HOME && targetState == State.PROCESSOR) {
            RobotContainer.currentState = State.PROCESSOR;
            CommandScheduler.getInstance().schedule(new HomeToProcessor(pivotSys, elevatorSys, extenderSys)); // home to processor
        } else if (RobotContainer.currentState == State.HOME && targetState == State.GROUND) {
            RobotContainer.currentState = State.GROUND;
            CommandScheduler.getInstance().schedule(new HomeToGround(pivotSys, elevatorSys, extenderSys)); // home to ground
        } else if (RobotContainer.currentState == State.CL1 && targetState == State.HOME && !RobotContainer.algaeMode) {
            RobotContainer.currentState = State.HOME;
            CommandScheduler.getInstance().schedule(new CL1ToHome(pivotSys, elevatorSys, extenderSys)); // cl1 to home
        } else if (RobotContainer.currentState == State.CL2 && targetState == State.HOME && !RobotContainer.algaeMode) {
            RobotContainer.currentState = State.HOME;
            CommandScheduler.getInstance().schedule(new CL2ToHome(pivotSys, elevatorSys, extenderSys)); // cl2 to home
        } else if (RobotContainer.currentState == State.CL3 && targetState == State.HOME && !RobotContainer.algaeMode) {
            RobotContainer.currentState = State.HOME;
            CommandScheduler.getInstance().schedule(new CL3ToHome(pivotSys, elevatorSys, extenderSys)); // cl3 to home
        } else if (RobotContainer.currentState == State.CL4 && targetState == State.HOME && !RobotContainer.algaeMode) {
            RobotContainer.currentState = State.HOME; 
            CommandScheduler.getInstance().schedule(new CL4ToHome(pivotSys, elevatorSys, extenderSys));// cl4 to home
        } else if (RobotContainer.currentState == State.GROUND && targetState == State.HOME) {
            RobotContainer.currentState = State.HOME;
            CommandScheduler.getInstance().schedule(new GroundToHome(pivotSys, elevatorSys, extenderSys)); // ground to home
        } else if (RobotContainer.currentState == State.PROCESSOR && targetState == State.HOME) {
            RobotContainer.currentState = State.HOME;
            CommandScheduler.getInstance().schedule(new ProcessorToHome(pivotSys, elevatorSys, extenderSys)); // processor to home
        } else if (RobotContainer.currentState == State.CL2 && targetState == State.HOME && RobotContainer.algaeMode) {
            RobotContainer.currentState = State.HOME;
            CommandScheduler.getInstance().schedule(new AL2ToHome(pivotSys, elevatorSys, extenderSys)); // al2 to home
        } else if (RobotContainer.currentState == State.CL3 && targetState == State.HOME && RobotContainer.algaeMode) {
            RobotContainer.currentState = State.HOME;
            CommandScheduler.getInstance().schedule(new AL3ToHome(pivotSys, elevatorSys, extenderSys)); // al3 to home
        } else if (RobotContainer.currentState == State.CL4 && targetState == State.HOME && RobotContainer.algaeMode) {
            RobotContainer.currentState = State.HOME;
            CommandScheduler.getInstance().schedule(new BargeToHome(pivotSys, elevatorSys, extenderSys)); // al3 to home
        } else if (RobotContainer.currentState == State.CLIMB && targetState == State.HOME && !RobotContainer.algaeMode) {
            RobotContainer.currentState = State.HOME;
            CommandScheduler.getInstance().schedule(new ClimbToHome(pivotSys, elevatorSys, extenderSys)); // climb to home
        } else if (RobotContainer.currentState == State.HOME && targetState == State.CLIMB && RobotContainer.algaeMode) {
            RobotContainer.currentState = State.CLIMB;
            CommandScheduler.getInstance().schedule(new HomeToClimb(pivotSys, elevatorSys, extenderSys)); // home to climb
        } else {
            CommandScheduler.getInstance().schedule(new DoNothings(winchSys));
        }
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
 