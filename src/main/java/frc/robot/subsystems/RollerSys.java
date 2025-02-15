package frc.robot.subsystems; 

import com.revrobotics.spark.SparkClosedLoopController; 
import com.revrobotics.spark.SparkFlex;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.CANDevices;


public class RollerSys extends SubsystemBase {
  private final SparkFlex rollerMtr;

  private final RelativeEncoder rollerEnc;

  private final SparkClosedLoopController rollerController;

  public RollerSys() {
    rollerMtr = new SparkFlex(CANDevices.rollerMtrID, MotorType.kBrushless);
    SparkFlexConfig rollerMtrSparkFlexConfig = new SparkFlexConfig();

    rollerMtrSparkFlexConfig.inverted(true);

    rollerMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);

    rollerMtrSparkFlexConfig.voltageCompensation(10);

    rollerMtrSparkFlexConfig.smartCurrentLimit(RollerConstants.maxRollerCurrentAmps);

    rollerEnc = rollerMtr.getEncoder();

    rollerMtrSparkFlexConfig.encoder.positionConversionFactor(RollerConstants.outputRevPerMtrRev);
    rollerMtrSparkFlexConfig.encoder.velocityConversionFactor(RollerConstants.outputRPMPerMtrRPM);

    rollerController = rollerMtr.getClosedLoopController();

    // Velocity PID
    // algaeRollerMtrSparkFlexConfig.closedLoop.p(AlgaeRollerConstants.kP);
    // algaeRollerMtrSparkFlexConfig.closedLoop.i(0.0);
    // algaeRollerMtrSparkFlexConfig.closedLoop.d(AlgaeRollerConstants.kD);
    // algaeRollerMtrSparkFlexConfig.closedLoop.velocityFF(AlgaeRollerConstants.feedForward);
    // MAXMotion
    rollerMtrSparkFlexConfig.closedLoop.maxMotion.maxAcceleration(RollerConstants.maxAccelRPMPerSec); // RPM per sec
    // algaeRollerMtrSparkFlexConfig.closedLoop.maxMotion.allowedClosedLoopError(0.0); // Rotations * positionconversionfactor

    rollerMtr.configure(
        rollerMtrSparkFlexConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {

  }

  public void setRPM(double rpm) {
    // MAXMotion
    rollerController.setReference(rpm, ControlType.kMAXMotionVelocityControl);
    rollerController.setReference(rpm, ControlType.kMAXMotionVelocityControl);
    // Velocity PID
    // alageRollerController.setReference(rpm, ControlType.kVelocity);
    // alageRollerController.setReference(rpm, ControlType.kVelocity);
  }

  public double getRPM() {
    return rollerEnc.getVelocity();
  }
}