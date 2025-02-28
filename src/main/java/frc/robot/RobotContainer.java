// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.ControllerConstants;
import frc.robot.Constants.RobotConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.commands.ArcadeDriveCmd;
import frc.robot.commands.climber.ClimberClimbCmd;
import frc.robot.commands.climber.ClimberHomeCmd;
import frc.robot.commands.climber.ClimberOutCmd;
import frc.robot.commands.coral.CoralOuttakeCmd;
import frc.robot.commands.coral.CoralStopCmd;
import frc.robot.commands.elevator.ElevatorCoralOneCmd;
import frc.robot.commands.elevator.ElevatorCoralThreeCmd;
import frc.robot.commands.elevator.ElevatorCoralTwoCmd;
import frc.robot.commands.elevator.ElevatorHomeCmd;
import frc.robot.commands.elevator.ElevatorManualCmd;
import frc.robot.commands.pivot.PivotGroundCmd;
import frc.robot.commands.pivot.PivotProcessorCmd;
import frc.robot.commands.pivot.PivotReefCmd;
import frc.robot.commands.pivot.PivotStowCmd;
import frc.robot.commands.roller.RollerIdleCmd;
import frc.robot.commands.roller.RollerIntakeCmd;
import frc.robot.commands.roller.RollerOuttakeCmd;
import frc.robot.subsystems.ClimberSys;
import frc.robot.subsystems.CoralSys;
import frc.robot.subsystems.ElevatorSys;
import frc.robot.subsystems.PivotSys;
import frc.robot.subsystems.RollerSys;
import frc.robot.subsystems.drive.PoseEstimator;
import frc.robot.subsystems.drive.SwerveDrive;
import frc.robot.subsystems.util.LimelightHelpers;

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
	// private final ExampleSubsystem exampleSubsystem = new ExampleSubsystem();

	private final SwerveDrive swerveDrive = new SwerveDrive();
	private final ElevatorSys elevatorSys = new ElevatorSys();
	private final PivotSys pivotSys = new PivotSys();
	private final ClimberSys climberSys = new ClimberSys();
	private final CoralSys coralSys = new CoralSys();
	private final RollerSys rollerSys = new RollerSys();

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
		NamedCommands.registerCommand("ElevatorUp", new ElevatorCoralThreeCmd(elevatorSys));
		NamedCommands.registerCommand("ElevatorDown", new ElevatorHomeCmd(elevatorSys));	
		NamedCommands.registerCommand("OuttakeCoral", new CoralOuttakeCmd(coralSys));
		NamedCommands.registerCommand("StopCoral", new CoralStopCmd(coralSys));
		NamedCommands.registerCommand("PivotStow", new PivotStowCmd(pivotSys));

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
		new PathPlannerAuto("Test");
		new PathPlannerAuto("RotationTest");
		new PathPlannerAuto("TranslationTestOne");

		// SysID routines
		// autoChooser.addOption("SysID Quasistatic Forward", SysIDRoutines.quasistaticForward(swerveDrive));
		// autoChooser.addOption("SysID Quasistatic Reverse", SysIDRoutines.quasistaticReverse(swerveDrive));
		// autoChooser.addOption("SysID Dynamic Forward", SysIDRoutines.dynamicForward(swerveDrive));
		// autoChooser.addOption("SysID Dynamic Reverse", SysIDRoutines.dynamicReverse(swerveDrive));

		autoChooser = AutoBuilder.buildAutoChooser("Do Nothing");

		SmartDashboard.putData("auto chooser", autoChooser);
		
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
		
		operatorController.x().onTrue(new ElevatorHomeCmd(elevatorSys));
		operatorController.a().onTrue(new ElevatorCoralOneCmd(elevatorSys));
		operatorController.b().onTrue(new ElevatorCoralTwoCmd(elevatorSys));
		operatorController.y().onTrue(new ElevatorCoralThreeCmd(elevatorSys));
  
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

		operatorController.povUp().onTrue(new ClimberOutCmd(climberSys));
		operatorController.povDown().onTrue(new ClimberClimbCmd(climberSys));
		operatorController.povLeft().onTrue(new ClimberHomeCmd(climberSys));
		operatorController.povRight().onTrue(new PivotGroundCmd(pivotSys));

		operatorController.leftBumper()
			.onTrue(new PivotReefCmd(pivotSys))
			.onTrue(new RollerIntakeCmd(rollerSys))
			.onFalse(new PivotStowCmd(pivotSys))
			.onFalse(new RollerIdleCmd(rollerSys));

		operatorController.rightBumper()
			.onTrue(new PivotProcessorCmd(pivotSys))
			.onTrue(new RollerOuttakeCmd(rollerSys))
			.onFalse(new PivotStowCmd(pivotSys))
			.onFalse(new RollerIdleCmd(rollerSys));

		// no beam breaks
		// operatorController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, ControllerConstants.tiggerPressedThreshold)
		// .onTrue(new CoralOuttakeCmd(coralSys)).onFalse(new CoralStopCmd(coralSys));		
		// operatorController.axisGreaterThan(XboxController.Axis.kLeftTrigger.value, ControllerConstants.tiggerPressedThreshold)
		// .onTrue(new CoralIntakeCmd(coralSys)).onFalse(new CoralStopCmd(coralSys));

		// beam breaks  
		operatorController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, ControllerConstants.tiggerPressedThreshold)
		.onTrue(new CoralOuttakeCmd(coralSys)).onFalse(new CoralStopCmd(coralSys));

		driverController.axisGreaterThan(XboxController.Axis.kRightTrigger.value, ControllerConstants.tiggerPressedThreshold)
			.onTrue(new PivotGroundCmd(pivotSys))
			.onTrue(new RollerIntakeCmd(rollerSys))
			.onFalse(new PivotStowCmd(pivotSys))
			.onFalse(new RollerIdleCmd(rollerSys));

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
		SmartDashboard.putNumber("FL CANcoder", swerveDrive.getCanCoderAngles()[0].getDegrees());
		SmartDashboard.putNumber("FR CANcoder", swerveDrive.getCanCoderAngles()[1].getDegrees());
		SmartDashboard.putNumber("BL CANcoder", swerveDrive.getCanCoderAngles()[2].getDegrees());
		SmartDashboard.putNumber("BR CANcoder", swerveDrive.getCanCoderAngles()[3].getDegrees());
		SmartDashboard.putNumber("pos-x", poseEstimator.get().getX());
		SmartDashboard.putNumber("pos-y", poseEstimator.get().getY());
		SmartDashboard.putNumber("left elevator enc", elevatorSys.getLeftCurrentPositionInches());
		SmartDashboard.putNumber("right elevator enc", elevatorSys.getRightCurrentPositionInches());
		SmartDashboard.putNumber("elevator position", elevatorSys.getCurrentPositionInches());
		SmartDashboard.putNumber("climber position", climberSys.getCurrentPositionDeg());
		SmartDashboard.putNumber("climber left position", climberSys.getLeftCurrentPositionDeg());
		SmartDashboard.putNumber("climber right position", climberSys.getRightCurrentPositionDeg());
		SmartDashboard.putNumber("pivot deg", pivotSys.getCurrentPositionDeg());
	}
}