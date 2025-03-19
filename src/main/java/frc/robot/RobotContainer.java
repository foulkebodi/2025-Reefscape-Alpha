// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.State;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.commands.AlgaeModeFalse;
import frc.robot.commands.AlgaeModeTrue;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.elevator.ElevatorBargeCmd;
import frc.robot.commands.elevator.ElevatorCL2Cmd;
import frc.robot.commands.elevator.ElevatorCL4Cmd;
import frc.robot.commands.elevator.ElevatorHomeCmd;
import frc.robot.commands.elevator.ElevatorManualCmd;
import frc.robot.commands.extender.ExtenderBargeCmd;
import frc.robot.commands.extender.ExtenderCL1Cmd;
import frc.robot.commands.extender.ExtenderCL23Cmd;
import frc.robot.commands.extender.ExtenderHomeCmd;
import frc.robot.commands.intake.IntakeIdleCmd;
import frc.robot.commands.intake.IntakeIntakeCmd;
import frc.robot.commands.intake.IntakeOuttakeCmd;
import frc.robot.commands.pivot.PivotBargeCmd;
import frc.robot.commands.pivot.PivotCL23Cmd;
import frc.robot.commands.pivot.PivotCL4Cmd;
import frc.robot.commands.pivot.PivotGroundCmd;
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
import frc.robot.commands.transitions.DoNothings;
import frc.robot.commands.transitions.GroundToAlgaeHome;
import frc.robot.commands.transitions.ProcessorToAlgaeHome;
import frc.robot.commands.util.ExampleCommand;
import frc.robot.commands.pivot.PivotChuteCmd;
import frc.robot.commands.pivot.PivotCoralHomeCmd;
import frc.robot.commands.winch.DoNothing;
import frc.robot.commands.winch.WinchInCmd;
import frc.robot.commands.winch.WinchOutCmd;
import frc.robot.subsystems.WinchSys;
import frc.robot.subsystems.IntakeSys;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.ExtenderSys;
import frc.robot.subsystems.PivotSys;
import frc.robot.subsystems.drive.PoseEstimator;
import frc.robot.subsystems.drive.SwerveDrive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

@SuppressWarnings("unused")

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
	// The robot's subsystems and commands are defined here...
	private final SwerveDrive swerveDrive = new SwerveDrive();
	private final ElevatorSys elevatorSys = new ElevatorSys();
	private final ExtenderSys extenderSys = new ExtenderSys();
	private final PivotSys pivotSys = new PivotSys();
	private final WinchSys winchSys = new WinchSys();
	private final IntakeSys intakeSys = new IntakeSys();
	private final StateMachine stateMachine = new StateMachine();

	private final PoseEstimator poseEstimator = new PoseEstimator(
		SwerveDriveConstants.kinematics,
		() -> swerveDrive.getHeading(),
		() -> swerveDrive.getModulePositions());

	private final CommandXboxController driverController = new CommandXboxController(ControllerConstants.kDriverControllerPort);
	private final CommandXboxController operatorController = new CommandXboxController(ControllerConstants.kOperatorControllerPort);
	public static State currentState = State.CHUTE;
    public static boolean algaeMode = false;

	// Initializes and populates the auto chooser with all the PathPlanner autos in the project.
	// The deafulat auto is "Do Nothing" and runs Commands.none(), which does nothing.
	private final SendableChooser<Command> autoChooser;

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// register named commands
		// NamedCommands.registerCommand("exampleCommand", new ExampleCommand(exampleSubsystem));
		NamedCommands.registerCommand("CoralHomeToCL4", new CoralHomeToCL4(pivotSys, elevatorSys, extenderSys));
		NamedCommands.registerCommand("CL4ToCoralHome", new CL4ToCoralHome(pivotSys, elevatorSys, extenderSys));
		NamedCommands.registerCommand("CoralHomeToChute", new CoralHomeToChute(pivotSys, elevatorSys, extenderSys));
		NamedCommands.registerCommand("ChuteToCoralHome", new ChuteToCoralHome(pivotSys, elevatorSys, extenderSys));

		NamedCommands.registerCommand("IntakeIntakeCmd", new IntakeIntakeCmd(intakeSys));
		NamedCommands.registerCommand("IntakeIdleCmd", new IntakeIdleCmd(intakeSys));
		NamedCommands.registerCommand("IntakeOuttakeCmd", new IntakeOuttakeCmd(intakeSys));

		// configure autobuilder
		AutoBuilder.configure(
			poseEstimator::get,
			poseEstimator::resetPose,
			swerveDrive::getRobotRelativeSpeeds, 
			(chassisSpeeds, feedforward) -> swerveDrive.driveRobotRelative(chassisSpeeds),
			new PPHolonomicDriveController(
				new PIDConstants(SwerveDriveConstants.autoTranslationKp, SwerveDriveConstants.autoTranslationKd),
				new PIDConstants(SwerveDriveConstants.autoRotationKp, SwerveDriveConstants.autoRotationKd)),
			new RobotConfig(RobotConstants.massKg, RobotConstants.momentOfInertiaKgMetersSq, 
				SwerveModuleConstants.moduleConfig, SwerveDriveConstants.kinematics.getModules()),
			() -> {
				var alliance = DriverStation.getAlliance();
				if (alliance.isPresent()) {
					return alliance.get() == DriverStation.Alliance.Red;
				} 
				return false;
			},
			swerveDrive);

		// competition autos
		new PathPlannerAuto("TwoPieceInsideLeft");
		new PathPlannerAuto("TwoPieceInsideRight");

		// test autos
		// new PathPlannerAuto("Test");
		// new PathPlannerAuto("RotationTest");
		// new PathPlannerAuto("TranslationTestOne");

		// SysID routines
		// autoChooser.addOption("SysID Quasistatic Forward", SysIDRoutines.quasistaticForward(swerveDrive));
		// autoChooser.addOption("SysID Quasistatic Reverse", SysIDRoutines.quasistaticReverse(swerveDrive));
		// autoChooser.addOption("SysID Dynamic Forward", SysIDRoutines.dynamicForward(swerveDrive));
		// autoChooser.addOption("SysID Dynamic Reverse", SysIDRoutines.dynamicReverse(swerveDrive));

		autoChooser = AutoBuilder.buildAutoChooser("Do Nothing");

		SmartDashboard.putData("auto chooser", autoChooser);

		// CameraServer.startAutomaticCapture("climber", 0);

		// Configure the trigger bindings
		configureBindings();
	}

	/**
	 * Use this method to define your trigger->command mappings. Triggers can be created via the
	 * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
	 * predicate, or via the named factories in {@link
	 * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
	 * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
	 * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
	 * joysticks}.
	 */
	private void configureBindings() {
		swerveDrive.setDefaultCommand(new ArcadeDriveCmd(
			() -> MathUtil.applyDeadband(driverController.getLeftY(), ControllerConstants.joystickDeadband),
			() -> MathUtil.applyDeadband(driverController.getLeftX(), ControllerConstants.joystickDeadband),
			() -> MathUtil.applyDeadband(driverController.getRightX(), ControllerConstants.joystickDeadband),
			true,
			swerveDrive,
			poseEstimator));
		
		// elevatorSys maunal control
		// elevatorSys.setDefaultCommand(new ElevatorManualCmd(
		// 	() -> MathUtil.applyDeadband((operatorController.getLeftY()), ControllerConstants.joystickDeadband), 
		// 	elevatorSys));

		// elevatorSys troubleshooting
		// operatorController.x().onTrue(new ElevatorHomeCmd(elevatorSys));
		// operatorController.a().onTrue(new ElevatorCL2Cmd(elevatorSys));
		// operatorController.b().onTrue(new ElevatorCL4Cmd(elevatorSys));
		// operatorController.y().onTrue(new ElevatorBargeCmd(elevatorSys));

		// pivotSys troubleshooting
		// operatorController.a().onTrue(new PivotCoralHomeCmd(pivotSys));
		// operatorController.b().onTrue(new PivotCL23Cmd(pivotSys));
		// operatorController.x().onTrue(new PivotChuteCmd(pivotSys));
		// operatorController.y().onTrue(new PivotBargeCmd(pivotSys));

		// extenderSys troubleshooting
		// operatorController.a().onTrue(new ExtenderCL1Cmd(extenderSys));
		// operatorController.b().onTrue(new ExtenderCL23Cmd(extenderSys));
		// operatorController.y().onTrue(new ExtenderBargeCmd(extenderSys));
		// operatorController.x().onTrue(new ExtenderHomeCmd(extenderSys));

		// competition setup
		operatorController.a().onTrue(new InstantCommand( () -> {
			System.out.println("running - a");
			getSequence(State.CL2);
		}, elevatorSys, extenderSys, pivotSys));
		operatorController.b().onTrue(new InstantCommand( () -> {
			System.out.println("running - b");
			getSequence(State.CL3);
		}, elevatorSys, extenderSys, pivotSys));
		operatorController.y().onTrue(new InstantCommand( () -> {
			System.out.println("running - y");
			getSequence(State.CL4);
		}, elevatorSys, extenderSys, pivotSys));
		operatorController.x().onTrue(new InstantCommand( () -> {
			System.out.println("running - x");
			getSequence(State.HOME);
		}, elevatorSys, extenderSys, pivotSys));
		
		operatorController.start().onTrue(getSequence(State.CL1));

		operatorController.leftBumper().onTrue(getSequence(State.PROCESSOR));
		operatorController.rightBumper().onTrue(getSequence(State.PROCESSOR));

		operatorController.povDown().onTrue(new WinchInCmd(winchSys));
		operatorController.povUp()
			.onTrue(new WinchOutCmd(winchSys))
			.onTrue(getSequence(State.CLIMB));

		operatorController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, ControllerConstants.tiggerPressedThreshold)
			.onTrue(getSequence(State.CHUTE))	
			.onTrue(new IntakeIntakeCmd(intakeSys))
			.onFalse(getSequence(State.HOME))
			.onFalse(new IntakeIdleCmd(intakeSys));
		
		operatorController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, ControllerConstants.tiggerPressedThreshold)
			.onTrue(new IntakeOuttakeCmd(intakeSys))
			.onFalse(new IntakeIdleCmd(intakeSys));

		operatorController.povLeft().onTrue(new AlgaeModeFalse());
		operatorController.povRight().onTrue(new AlgaeModeTrue());

		driverController.start().onTrue(Commands.runOnce(() -> poseEstimator.resetHeading()));

		driverController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, ControllerConstants.tiggerPressedThreshold)
		.onTrue(getSequence(State.GROUND))
		.onTrue(new IntakeIntakeCmd(intakeSys))
		.onFalse(new IntakeIdleCmd(intakeSys))
		.onTrue(getSequence(State.HOME));
	}

	/**
	 * Use this to pass the autonomous command to the main {@link Robot} class.
	 *
	 * @return the command to run in autonomous
	 */
	public Command getAutonomousCommand() {
		return autoChooser.getSelected();
	}

	public void updateDashboard() {
		// drive base and pose information
		SmartDashboard.putNumber("FL CANcoder", swerveDrive.getCanCoderAngles()[0].getDegrees());
		SmartDashboard.putNumber("FR CANcoder", swerveDrive.getCanCoderAngles()[1].getDegrees());
		SmartDashboard.putNumber("BL CANcoder", swerveDrive.getCanCoderAngles()[2].getDegrees());
		SmartDashboard.putNumber("BR CANcoder", swerveDrive.getCanCoderAngles()[3].getDegrees());
		SmartDashboard.putNumber("pos-x", poseEstimator.get().getX());
		SmartDashboard.putNumber("pos-y", poseEstimator.get().getY());
		SmartDashboard.putBoolean("is autonomous", DriverStation.isAutonomous());

		// elevatorSys info
		SmartDashboard.putNumber("left elevator position", elevatorSys.getLeftCurrentPositionInches());
		SmartDashboard.putNumber("right elevator position", elevatorSys.getRightCurrentPositionInches());
		SmartDashboard.putNumber("elevator position", elevatorSys.getCurrentPositionInches());
		SmartDashboard.putBoolean("elevator at target", elevatorSys.isAtTarget());
		SmartDashboard.putNumber("elevator error inches", elevatorSys.getErrorInches());
		SmartDashboard.putNumber("elevator target position", elevatorSys.getTargetInches());

		// climber/winch info
		SmartDashboard.putNumber("winch position", winchSys.getWinchCurrentPositionDeg());
		SmartDashboard.putNumber("winch power", winchSys.getWinchPower());

		// pivotSys info
		SmartDashboard.putNumber("pivot deg", pivotSys.getCurrentPositionDeg());
		SmartDashboard.putNumber("pivot target deg", pivotSys.getTargetDeg());
		SmartDashboard.putNumber("pivot error deg", pivotSys.getErrorDeg());
		SmartDashboard.putBoolean("pivot at target", pivotSys.isAtTarget());

		// intake info
		SmartDashboard.putBoolean("beam break", intakeSys.getFilteredBeamBreak());
		SmartDashboard.putNumber("intake target power", intakeSys.getTargetPower());
		SmartDashboard.putNumber("intake output current amps", intakeSys.getFilteredOutputCurrent());
		SmartDashboard.putNumber("intake time millis", intakeSys.getIntakeCurrentTimeMillis());
		SmartDashboard.putBoolean("intaking", intakeSys.getIntaking());
		SmartDashboard.putBoolean("outtaking", intakeSys.getOuttaking());

		// extenderSys info
		SmartDashboard.putNumber("extender position inches", extenderSys.getCurrentPositionInches());
		SmartDashboard.putNumber("extender target position inches", extenderSys.getTargetInches());
		SmartDashboard.putBoolean("extender at target", extenderSys.isAtTarget());
		SmartDashboard.putNumber("extender error inches", extenderSys.getErrorInches());
		SmartDashboard.putNumber("extender target power", extenderSys.getTargetPower());

		// state info
		SmartDashboard.putString("current state", getStateAsString(currentState));
		SmartDashboard.putBoolean("algae mode", algaeMode);

	}

	public SequentialCommandGroup getSequence(State targetState) {
        if(currentState == State.HOME && targetState == State.HOME && !algaeMode) {
            currentState = State.HOME;
			System.out.println("running - algae home to coral home");
            return new AlgaeHomeToCoralHome(pivotSys, elevatorSys, extenderSys); // algae home to coral home
        } else if (currentState == State.HOME && targetState == State.HOME && algaeMode) {
            currentState = State.HOME;
			System.out.println("running - coral home to algae home");
            return new CoralHomeToAlgaeHome(pivotSys, elevatorSys, extenderSys); // coral home to algae home
        } else if (currentState == State.HOME && targetState == State.CHUTE && !algaeMode) {
			currentState = State.HOME;
			System.out.println("running - coral home to chute");
            return new CoralHomeToChute(pivotSys, elevatorSys, extenderSys); // coral home to chute
        } else if (currentState == State.HOME && targetState == State.CL1 && !algaeMode) {
            currentState = State.CL1;
			System.out.println("running - coral home to cl1");
            return new CoralHomeToCL1(pivotSys, elevatorSys, extenderSys); // coral home to cl1
        } else if (currentState == State.HOME && targetState == State.CL2 && !algaeMode) {
            currentState = State.CL2;
			System.out.println("running - coral home to cl2");
            return new CoralHomeToCL2(pivotSys, elevatorSys, extenderSys); // coral home to cl2
        } else if (currentState == State.HOME && targetState == State.CL3 && !algaeMode) {
            currentState = State.CL3;
            return new CoralHomeToCL3(pivotSys, elevatorSys, extenderSys); // coral home to cl3
        } else if (currentState == State.HOME && targetState == State.CL4 && !algaeMode) {
            currentState = State.CL4;
            return new CoralHomeToCL4(pivotSys, elevatorSys, extenderSys); // coral home to cl4
        } else if (currentState == State.HOME && targetState == State.CL2 && algaeMode) {
            currentState = State.CL2;
            return new AlgaeHomeToAL2(pivotSys, elevatorSys, extenderSys); // algae home to al2
        } else if (currentState == State.HOME && targetState == State.CL3 && algaeMode) {
            currentState = State.CL3;
            return new AlgaeHomeToAL3(pivotSys, elevatorSys, extenderSys); // algae home to al3
        } else if (currentState == State.HOME && targetState == State.CL4 && algaeMode) {
            currentState = State.CL4;
            return new AlgaeHomeToBarge(pivotSys, elevatorSys, extenderSys); // algae home to barge
        } else if (currentState == State.HOME && targetState == State.PROCESSOR && algaeMode) {
            currentState = State.PROCESSOR;
            return new AlgaeHomeToProcessor(pivotSys, elevatorSys, extenderSys); // algae home to processor
        } else if (currentState == State.HOME && targetState == State.GROUND && algaeMode) {
            currentState = State.GROUND;
            return new AlgaeHomeToGround(pivotSys, elevatorSys, extenderSys); // algae home to ground
        } else if (currentState == State.CL1 && targetState == State.HOME && !algaeMode) {
            currentState = State.HOME;
            return new CL1ToCoralHome(pivotSys, elevatorSys, extenderSys); // cl1 to  coral home
        } else if (currentState == State.CL2 && targetState == State.HOME && !algaeMode) {
            currentState = State.HOME;
            return new CL23ToCoralHome(pivotSys, elevatorSys, extenderSys); // cl2 to coral home
        } else if (currentState == State.CL3 && targetState == State.HOME && !algaeMode) {
            currentState = State.HOME;
            return new CL23ToCoralHome(pivotSys, elevatorSys, extenderSys); // cl3 to coral home
        } else if (currentState == State.CL4 && targetState == State.HOME && !algaeMode) {
            currentState = State.HOME; 
            return new CL4ToCoralHome(pivotSys, elevatorSys, extenderSys);// cl4 to coral home
        } else if (currentState == State.GROUND && targetState == State.HOME && algaeMode) {
            currentState = State.HOME;
            return new GroundToAlgaeHome(pivotSys, elevatorSys, extenderSys); // ground to algae home
        } else if (currentState == State.PROCESSOR && targetState == State.HOME && algaeMode) {
            currentState = State.HOME;
            return new ProcessorToAlgaeHome(pivotSys, elevatorSys, extenderSys); // processor to algae home
        } else if (currentState == State.CL4 && targetState == State.HOME && algaeMode) {
            currentState = State.HOME;
            return new BargeToAlgaeHome(pivotSys, elevatorSys, extenderSys); // barge to algae home
        } else if (currentState == State.CL2 && targetState == State.HOME && algaeMode) {
            currentState = State.HOME;
            return new AL2ToAlgaeHome(pivotSys, elevatorSys, extenderSys); // al2 to algae home
        } else if (currentState == State.CL3 && targetState == State.HOME && algaeMode) {
            currentState = State.HOME;
            return new AL3ToAlgaeHome(pivotSys, elevatorSys, extenderSys); // al3 to algae home
        } else if (currentState == State.CHUTE && targetState == State.HOME && !algaeMode) {
            currentState = State.HOME;
            return new ChuteToCoralHome(pivotSys, elevatorSys, extenderSys); // chute to coral home
        } else if (currentState == State.HOME && targetState == State.CLIMB && algaeMode) {
            currentState = State.HOME;
            return new AlgaeHomeToClimb(pivotSys, elevatorSys, extenderSys); // algae home to climb
        } else if (currentState == State.HOME && targetState == State.CLIMB && !algaeMode) {
            currentState = State.HOME;
            return new CoralHomeToClimb(pivotSys, elevatorSys, extenderSys); // coral home to climb
        } else if (currentState == State.CLIMB && targetState == State.HOME && !algaeMode) {
            currentState = State.HOME;
            return new ClimbToCoralHome(pivotSys, elevatorSys, extenderSys); // climb to coral home
        } else if (currentState == State.CLIMB && targetState == State.HOME && algaeMode) {
            currentState = State.HOME;
            return new ClimbToAlgaeHome(pivotSys, elevatorSys, extenderSys); // climb to algae home
        } else {
            return new DoNothings(winchSys);
        }
    }

	public String getStateAsString(State state) {
		if(currentState == State.HOME && algaeMode) {
            return "AH";
        } else if(state == State.AL2) {
            return "AL2";
        } else if(state == State.AL3) {
            return "AL3";
        } else if(state == State.BARGE) {
            return "BARGE";
        } else if(state == State.HOME && !algaeMode) {
            return "CH";
        } else if(state == State.CL1) {
            return "CL1";
        } else if(state == State.CL2) {
            return "CL2";
        } else if(state == State.CL3) {
            return "CL3";
        } else if(state == State.CL4) {
            return "CL4";
        } else if(state == State.GROUND) {
            return "GROUND";
        } else if(state == State.CHUTE) {
            return "INTAKING";
        } else if(state == State.PROCESSOR) {
            return "PROCESSOR";
        } else {
            return "null";
        }
    }
}