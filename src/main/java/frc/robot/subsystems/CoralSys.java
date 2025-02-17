package frc.robot.subsystems; 

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.CoralConstants;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;


public class CoralSys extends SubsystemBase {
  private final SparkFlex rightCoralMtr;

  private final RelativeEncoder rightCoralEnc;

  private final SparkClosedLoopController rightCoralController;

  private final DigitalInput backBeamBreak;
  private final DigitalInput frontBeamBreak;

  private double targetRPM;

public CoralSys() {
    rightCoralMtr = new SparkFlex(CANDevices.rightCoralMtrID, MotorType.kBrushless);
    SparkFlexConfig rightCoralSparkFlexConfig = new SparkFlexConfig();

    rightCoralSparkFlexConfig.inverted(false);

    rightCoralSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);

    rightCoralSparkFlexConfig.voltageCompensation(10);

    rightCoralSparkFlexConfig.smartCurrentLimit(CoralConstants.maxCoralCurrentAmps);

    rightCoralEnc = rightCoralMtr.getEncoder();

   
    rightCoralSparkFlexConfig.encoder.positionConversionFactor(CoralConstants.outputRevPerMtrRev);
    rightCoralSparkFlexConfig.encoder.velocityConversionFactor(CoralConstants.outputRPMPerMtrRPM);

    rightCoralController = rightCoralMtr.getClosedLoopController();

    // Velocity PID
    // rightCoralSparkFlexConfig.closedLoop.p(AlgaeRollerConstants.kP);
    // rightCoralSparkFlexConfig.closedLoop.i(0.0);
    // rightCoralSparkFlexConfig.closedLoop.d(AlgaeRollerConstants.kD);
    // rightCoralSparkFlexConfig.closedLoop.velocityFF(AlgaeRollerConstants.feedForward);

    // MAXMotion
    rightCoralSparkFlexConfig.closedLoop.maxMotion.maxAcceleration(CoralConstants.maxAccelRPMPerSec); // RPM per sec
    // rightCoralSparkFlexConfig.closedLoop.maxMotion.allowedClosedLoopError(0.0); // Rotations * positionconversionfactor
    
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
      // } else if (RPM = outtakerpm) {
      // rollers outtake
      // } else {
      // rollers at intake rpm
      // }
      // true is broken, false is unbroken
    if (backBeamBreak.get() == false && frontBeamBreak.get() == true) {  
      rightCoralController.setReference(0.0, ControlType.kMAXMotionVelocityControl);
    } else if (targetRPM == CoralConstants.outtakeRPM) {
      rightCoralController.setReference(CoralConstants.outtakeRPM, ControlType.kMAXMotionVelocityControl);
    } else if (DriverStation.isDisabled()) {
      rightCoralController.setReference(0.0, ControlType.kMAXMotionVelocityControl);
    } else {
      rightCoralController.setReference(CoralConstants.intakeRPM, ControlType.kMAXMotionVelocityControl);
    }
  }

  // public void setRPM(double rpm) {
    // MAXMotion
    // leftCoralController.setReference(rpm, ControlType.kMAXMotionVelocityControl);
    // rightCoralController.setReference(rpm, ControlType.kMAXMotionVelocityControl);
    // Velocity PID
    // leftCoralController.setReference(rpm, ControlType.kVelocity);
    // rightCoralController.setReference(rpm, ControlType.kVelocity);
  // }

  public void settargetRPM(double rpm) {
    targetRPM = rpm;
  }

  public double getRPM() {
      return (rightCoralMtr.getEncoder().getVelocity()) / 2;
  }
}