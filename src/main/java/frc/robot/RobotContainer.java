// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.State;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.elevator.ElevatorManualCmd;
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
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

	// Initializes and populates the auto chooser with all the PathPlanner autos in the project.
	// The deafulat auto is "Do Nothing" and runs Commands.none(), which does nothing.
	private final SendableChooser<Command> autoChooser;

	/** The container for the robot. Contains subsystems, OI devices, and commands. */
	public RobotContainer() {
		// register named commands
		// NamedCommands.registerCommand("exampleCommand", new ExampleCommand(exampleSubsystem));

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
		
		elevatorSys.setDefaultCommand(new ElevatorManualCmd(
			() -> MathUtil.applyDeadband((operatorController.getLeftY()), ControllerConstants.joystickDeadband), 
			elevatorSys));

		// elevator troubleshooting
		// operatorController.x().onTrue(new ElevatorHomeCmd(elevatorSys));
		// operatorController.a().onTrue(new ElevatorCoralOneCmd(elevatorSys));
		// operatorController.b().onTrue(new ElevatorCoralTwoCmd(elevatorSys));
		// operatorController.y().onTrue(new ElevatorCoralThreeCmd(elevatorSys));

		// pivot troubleshooting
		// operatorController.a().onTrue(new PivotGroundCmd(pivotSys));
		// operatorController.b().onTrue(new PivotProcessorCmd(pivotSys));
		// operatorController.x().onTrue(new PivotStowCmd(pivotSys));
		// operatorController.y().onTrue(new PivotReefCmd(pivotSys));

		// roller troubleshooting
		// operatorController.a().onTrue(new RollerIntakeCmd(rollerSys));
		// operatorController.b().onTrue(new RollerOuttakeCmd(rollerSys));
		// operatorController.x().onTrue(new RollerIdleCmd(rollerSys));
		// operatorController.x().onTrue(new AutoPlaceCoralCmd(coralSys));

		operatorController.povDown().onTrue(new WinchInCmd(winchSys));
		operatorController.povUp().onTrue(new WinchOutCmd(winchSys));

		operatorController.leftBumper().onTrue(stateMachine.getSequence(State.AH));

		driverController.start().onTrue(Commands.runOnce(() -> poseEstimator.resetHeading()));
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

		// elevator info
		SmartDashboard.putNumber("left elevator position", elevatorSys.getLeftCurrentPositionInches());
		SmartDashboard.putNumber("right elevator position", elevatorSys.getRightCurrentPositionInches());
		SmartDashboard.putNumber("elevator position", elevatorSys.getCurrentPositionInches());
		SmartDashboard.putBoolean("elevator at target", elevatorSys.isAtTarget());
		SmartDashboard.putNumber("elevator position", elevatorSys.getTargetInches());

		// climber/winch info
		SmartDashboard.putNumber("winch position", winchSys.getWinchCurrentPositionDeg());
		SmartDashboard.putNumber("winch power", winchSys.getWinchPower());

		// pivot info
		SmartDashboard.putNumber("algae pivot deg", pivotSys.getCurrentPositionDeg());
		SmartDashboard.putNumber("pivot target deg", pivotSys.getTargetDeg());

		// intake info
		SmartDashboard.putBoolean("beam break", intakeSys.getBeamBreak());
		SmartDashboard.putNumber("intake target power", intakeSys.getTargetPower());
		SmartDashboard.putNumber("intake output current amps", intakeSys.getOutputCurrent());
		SmartDashboard.putNumber("intake time millis", intakeSys.getCurrentTimeMillis());

		// extender info
		SmartDashboard.putNumber("extender position inches", extenderSys.getCurrentPositionInches());
		SmartDashboard.putNumber("extender error inches", extenderSys.getErrorInches());

		// state info

	}
}