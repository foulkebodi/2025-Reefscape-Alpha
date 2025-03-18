// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class RobotConstants {
        // Set to the current the weight of the robot, including the battery and bumpers.
        public static final double massKg = 52.41;

        // Set the frame dimensions of the robot.
        public static final double robotWidthMeters = Units.inchesToMeters(27.0);
        public static final double robotLengthMeters = Units.inchesToMeters(27.0); 

        // Moment of inertia of a uniform-mass slab with the axis of rotation centered and perpendicular to the slab
        // This should be a reasonable approximation of the robot's MOI
        public static final double momentOfInertiaKgMetersSq = massKg * (Math.pow(robotWidthMeters, 2) + Math.pow(robotLengthMeters, 2)) / 12;
    }

	public static class ControllerConstants {
		public static final int kDriverControllerPort = 0;
        public static final int kOperatorControllerPort = 1;

		public static final double joystickDeadband = 0.075;
        public static final double tiggerPressedThreshold = 0.25;
	}

	public static class CANDevices {
		public static final int pigeonID = 14;

		public static final int flModuleCANCoderID = 2;
		public static final int flModuleDriveMtrID = 6;
		public static final int flModuleSteerMtrID = 10;

		public static final int frModuleCANCoderID = 3;
		public static final int frModuleDriveMtrID = 7;
		public static final int frModuleSteerMtrID = 11;

		public static final int blModuleCANCoderID = 4;
		public static final int blModuleDriveMtrID = 8;
		public static final int blModuleSteerMtrID = 12;

		public static final int brModuleCANCoderID = 5;
		public static final int brModuleDriveMtrID = 9;
		public static final int brModuleSteerMtrID = 13;

        public static final int topElevatorMtrID = 15;
        public static final int bottomElevatorMtrID = 16;

        public static final int pivotMtrID = 17;
        public static final int pivotEncPortID = 0; // RIO DIO

        public static final int extenderMtrID = 19;

        public static final int intakeMtrID = 18;

        public static final int beamBreakPort = 1;
       
        public static final int winchMtrID = 20;
	}

    public static class SwerveModuleConstants {
        // Tune the below PID and FF values using the SysID routines.
        public static final double driveKp = 0.12; 
        public static final double driveKd = 0.0;

        public static final double steerKp = 0.37431;
        public static final double steerKd = 0.27186;

        public static final double driveKsVolts = 0.667;
        public static final double driveKvVoltSecsPerMeter = 2.44;
        public static final double driveKaVoldSecsPerMeterSq = 0.0;

        public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(driveKsVolts, driveKvVoltSecsPerMeter, driveKaVoldSecsPerMeterSq);

        // You may want to change this value depending on your breakers and the current usage of the rest of your robot.
        public static final int driveCurrentLimitAmps = 80;

        // This number may have to be adjusted depending on what wheels you use.
        public static final double wheelRadiusMeters = Units.inchesToMeters(1.9);

        // Set this value to the coefficient of friction of your wheels.
        public static final double wheelCoefficientOfFriction = 1.5;

        public static final double driveGearReduction = (16.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);

        public static final double driveMetersPerEncRev = driveGearReduction * 2.0 * wheelRadiusMeters * Math.PI;

        public static final double driveMetersPerSecPerEncRPM = driveMetersPerEncRev / 60.0;

        public static final double steerGearReduction = (15.0 / 32.0) * (10.0 / 60.0); 

        public static final double steerRadiansPerEncRev = steerGearReduction * 2.0 * Math.PI;

        public static final double steerRadiansPerSecPerEncRPM = steerRadiansPerEncRev / 60.0;
        
        public static final double driveFreeSpeedMetersPerSec = Units.feetToMeters(20.1);

        public static final double driveFreeSpeedRadPerSec = driveFreeSpeedMetersPerSec / wheelRadiusMeters;

        public static final double driveNominalOperatingVoltage = 12.0;
        public static final double driveStallTorqueNewtonMeters = 3.6 / driveGearReduction; // Motor's stall torque times gear ratio
        public static final double driveStallCurrentAmps = 211.0;
        public static final double driveFreeCurrentAmps = 3.6;

        public static final ModuleConfig moduleConfig = new ModuleConfig(
            wheelRadiusMeters, driveFreeSpeedMetersPerSec, wheelCoefficientOfFriction, 
            new DCMotor(driveNominalOperatingVoltage, driveStallTorqueNewtonMeters, driveStallCurrentAmps, driveFreeCurrentAmps, driveFreeSpeedRadPerSec, 1),
            driveCurrentLimitAmps, 4);
    }

	public static class SwerveDriveConstants {
        public static final Rotation2d flModuleOffset = Rotation2d.fromDegrees(32.08);
        public static final Rotation2d frModuleOffset = Rotation2d.fromDegrees(164.17);
        public static final Rotation2d blModuleOffset = Rotation2d.fromDegrees(71.68);
        public static final Rotation2d brModuleOffset = Rotation2d.fromDegrees(116.9);

		// Set these dimensions for the distance between the center of each wheel.
        // Note that these values are different from the robot's overall dimenstions.
		public static final double chassisLengthMeters = Units.inchesToMeters(21.75); // 27 inch frame
        public static final double chassisWidthMeters = Units.inchesToMeters(21.75); // 27 inch frame

        public static final double chassisRadiusMeters = Math.hypot(chassisLengthMeters, chassisWidthMeters);

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(chassisWidthMeters / 2.0, chassisLengthMeters / 2.0),  // front left
            new Translation2d(chassisWidthMeters / 2.0, -chassisLengthMeters / 2.0), // front right
            new Translation2d(-chassisWidthMeters / 2.0, chassisLengthMeters / 2.0), // back left
            new Translation2d(-chassisWidthMeters / 2.0, -chassisLengthMeters / 2.0) // back right
        );

        public static final double maxAttainableSpeedMetersPerSec = Units.feetToMeters(20.1 * 0.9);
        public static final double maxAttainableRotationRadPerSec = 13.4;

        public static final double skewCompensationRatioOmegaPerTheta = 0.1;

        // Tune the below PID values using the SysID routines.
        public static final double autoTranslationKp = 6.0;
        public static final double autoTranslationKd = 0.0;

        public static final double autoRotationKp = 8.0;
        public static final double autoRotationKd = 0.0;
    }

    public static class ElevatorConstants {
        public static final int maxElevatorCurrentAmps = 50;

        public static final double gearRatio = 6.0;

        public static final double kP = 0.065;
        public static final double kD = 0.0;

        public static final double sprocketToothCount = 22.5; 
        public static final double chainPitch = 0.25; // inches

        public static final double inchesPerMtrRev = ((sprocketToothCount * chainPitch * 2.0) / gearRatio);
        public static final double inchesPerSecPerRPM = ((sprocketToothCount * chainPitch * 2.0) / gearRatio * 60.0);

        public static final double freeSpeedInchesPerSec = inchesPerMtrRev * 6784.0 / 60.0;

        public static final double maxVelInchesPerSec = 60.0;

        public static final double maxAccelInchesPerSecSq = 100.0;

        public static final double maxManualInchesPerSec = 100.0;

        public static final double maxManualInchesPerSecSq = 100.0;

        public static final double homeInches = 0.0;
        public static final double CL2Inches = 10.0;
        public static final double CL3Inches = 20.0;
        public static final double CL4Inches = 30.0;
        public static final double bargeInches = 47.0;

        public static final float lowerLimitInches = 0f;

        public static final float upperLimitInches = 47.7f; // 47.7 theoretical max

        public static final double toleranceInches = 0.5;
    }

    public static class ExtenderConstants {
        public static final int maxExtenderCurrentAmps = 40;

        public static final double gearRatio = 15.0;

        public static final double kP = 0.425;
        public static final double kD = 0.0045;

        public static final double sprocketToothCount = 14; 
        public static final double chainPitch = 0.375; // inches  

        public static final double inchesPerMtrRev = chainPitch * sprocketToothCount / gearRatio;
        public static final double inchesPerSecPerRPM = chainPitch * sprocketToothCount / gearRatio * 60.0;

        public static final double freeSpeedInchesPerSec = inchesPerMtrRev * 6784.0 / 60.0;

        public static final double maxVelInchesPerSec = freeSpeedInchesPerSec;

        public static final double maxAccelInchesPerSecSq = 200.0;

        public static final double homeInches = 0.0;
        public static final double CL1Inches = 5.0;
        public static final double CL23Inches = 10.0;
        public static final double CL4Inches = 0.0;
        public static final double ALInches = 0.0;
        public static final double bargeInches = 13.0;

        public static final float lowerLimitInches = 0f;

        public static final float upperLimitInches = 13f;

        public static final double toleranceInches = 0.1;
    }

    public class PivotConstants {
        public static final int maxPivotCurrentAmps = 50;

        public static final double gearRatio = 75.0;

        public static final double kP = 0.012;
        public static final double kD = 0.000;

        public static final double degPerEncRev = 360.0 / gearRatio;
        public static final double degPerSecPerRPM = 360.0 / (60.0 * gearRatio);

        public static final double freeSpeedRPM = 6784.0 / gearRatio;

        public static final double maxVelDegPerSec = 150.0;

        public static final double maxAccelDegPerSecSq = 100.0;

        public static final double maxManualDegPerSec = 100.0;

        public static final double maxManualDegPerSecSq = 375.0;

        public static final double chuteDeg = 5.0;
        public static final double coralHomeDeg = 10.0;
        public static final double algaeHomeDeg = 15.0;
        public static final double CL1Deg = 55.0;
        public static final double CL23Deg = 45.0;
        public static final double CL4Deg = 70.0;
        public static final double ALDeg = 85.0;
        public static final double processorDeg = 70.0;
        public static final double groundDeg = 85.0;
        public static final double bargeDeg = 60.0;

        public static final float lowerLimitDeg = 5f;

        public static final float upperLimitDeg = 90f;

        public static final double toleranceDeg = 1.0;

        public static final double absPivotEncOffsetDeg = 0.0 + chuteDeg;
    }

    public class IntakeConstants {
        public static final int maxIntakeCurrentAmps = 35;
        
        public static final double outtakePower = 1.0;

        public static final double intakePower = -0.3;

        public static final double idlePower = -0.1;

        public static final double idleOutPower = 0.07;

        public static final double waitSeconds = 0.5;

        public static final double currentThreshold = 35;

        public static final double currentDebounceTime = 0.33;
        public static final double beamBreakDebounceTime = 0.1;

        public static final int filterSize = 5;
    }

    public static class WinchConstants {
        public static final int maxWinchCurrentAmps = 100;

        public static final double gearRatio = 125.0;

        public static final double kP = 0.028;
        public static final double kD = 0.0001;

        public static final double degPerEncRev = 360.0 / gearRatio;
        public static final double degPerSecPerRPM = 360.0 / (60.0 * gearRatio);
        
        public static final double maxVelDegPerSec = 150.0;

        public static final double maxAccelDegPerSecSq = 150.0; 

        public static final double inPresetDeg = -1000;

        public static final double outPresetDeg = 5000;

        public static final float lowerLimitDeg = -1000f;

        public static final float upperLimitDeg = 5000f;
    }

    public class VisionConstants {
        public static final String frontLimelightName = "limelight-front";

        public static final String backLimelightName = "limelight-back";
    }

    public enum State {
        CHUTE,
        HOME,
        CL1,
        CL2,
        CL3,
        CL4,
        AL2,
        AL3,
        PROCESSOR,
        GROUND,
        BARGE,
        CLIMB
    }
}
