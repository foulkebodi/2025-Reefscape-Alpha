package frc.robot.subsystems; 

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.hal.CANAPITypes.CANDeviceType;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ElevatorConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;


public class CoralSys extends SubsystemBase {
  private final SparkFlex rightCoralMtr;
  private final SparkFlex leftCoralMtr;

  private final RelativeEncoder rightCoralEnc;
  private final RelativeEncoder leftCoralEnc;

  private final SparkClosedLoopController leftCoralController;
  private final SparkClosedLoopController rightCoralController;

  private final DigitalInput backBeamBreak;
  private final DigitalInput frontBeamBreak;

public CoralSys() {
    rightCoralMtr = new SparkFlex(CANDevices.rightCoralMtrID, MotorType.kBrushless);
    SparkFlexConfig rightCoralSparkFlexConfig = new SparkFlexConfig();
    leftCoralMtr = new SparkFlex(CANDevices.leftCoralMtrID, MotorType.kBrushless);
    SparkFlexConfig leftCoralSparkFlexConfig = new SparkFlexConfig();

    rightCoralSparkFlexConfig.inverted(false);
    leftCoralSparkFlexConfig.inverted(true);

    rightCoralSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
    leftCoralSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);

    rightCoralSparkFlexConfig.voltageCompensation(10);
    leftCoralSparkFlexConfig.voltageCompensation(10);

    rightCoralSparkFlexConfig.smartCurrentLimit(RollerConstants.maxRollerCurrentAmps);
    leftCoralSparkFlexConfig.smartCurrentLimit(RollerConstants.maxRollerCurrentAmps);

    leftCoralEnc = leftCoralMtr.getEncoder();
    rightCoralEnc = rightCoralMtr.getEncoder();

    leftCoralSparkFlexConfig.encoder.positionConversionFactor(RollerConstants.outputRevPerMtrRev);
    leftCoralSparkFlexConfig.encoder.velocityConversionFactor(RollerConstants.outputRPMPerMtrRPM);
    rightCoralSparkFlexConfig.encoder.positionConversionFactor(RollerConstants.outputRevPerMtrRev);
    rightCoralSparkFlexConfig.encoder.velocityConversionFactor(RollerConstants.outputRPMPerMtrRPM);

    leftCoralController = leftCoralMtr.getClosedLoopController();
    rightCoralController = rightCoralMtr.getClosedLoopController();

    // Velocity PID
    // rightCoralSparkFlexConfig.closedLoop.p(AlgaeRollerConstants.kP);
    // rightCoralSparkFlexConfig.closedLoop.i(0.0);
    // rightCoralSparkFlexConfig.closedLoop.d(AlgaeRollerConstants.kD);
    // rightCoralSparkFlexConfig.closedLoop.velocityFF(AlgaeRollerConstants.feedForward);

    // MAXMotion
    leftCoralSparkFlexConfig.closedLoop.maxMotion.maxAcceleration(RollerConstants.maxAccelRPMPerSec); // RPM per sec
    rightCoralSparkFlexConfig.closedLoop.maxMotion.maxAcceleration(RollerConstants.maxAccelRPMPerSec); // RPM per sec
    // rightCoralSparkFlexConfig.closedLoop.maxMotion.allowedClosedLoopError(0.0); // Rotations * positionconversionfactor

    leftCoralMtr.configure(
        leftCoralSparkFlexConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);    
    rightCoralMtr.configure(
        rightCoralSparkFlexConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

  backBeamBreak = new DigitalInput(CANDevices.backBeamBreakPort);
  frontBeamBreak = new DigitalInput(CANDevices.frontBeamBreakPort);
  }

  @Override
  public void periodic() {
      // if (bbb = 0 and fbb = 1) {
      // rollers stopped
      // } else {
      // rollers at intake rpm
      // }
  }

  public void setRPM(double rpm) {

    // MAXMotion
    // leftCoralController.setReference(rpm, ControlType.kMAXMotionVelocityControl);
    // rightCoralController.setReference(rpm, ControlType.kMAXMotionVelocityControl);
    // Velocity PID
    // leftCoralController.setReference(rpm, ControlType.kVelocity);
    // rightCoralController.setReference(rpm, ControlType.kVelocity);
  }

  public double getRPM() {
      return (leftCoralMtr.getEncoder().getVelocity() + rightCoralMtr.getEncoder().getVelocity()) / 2;
  }
}