package frc.robot.subsystems; 

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.CoralConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;


public class CoralSys extends SubsystemBase {
  private final SparkFlex CoralMtr;

  // private final RelativeEncoder CoralEnc;

  // private final SparkClosedLoopController CoralController;

  private final DigitalInput backBeamBreak;
  private final DigitalInput frontBeamBreak;

  // private double power;

  private boolean isOuttaking = false;

public CoralSys() {
    CoralMtr = new SparkFlex(CANDevices.CoralMtrID, MotorType.kBrushless);
    SparkFlexConfig CoralSparkFlexConfig = new SparkFlexConfig();

    CoralSparkFlexConfig.inverted(true);

    CoralSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);

    CoralSparkFlexConfig.voltageCompensation(10);

    CoralSparkFlexConfig.smartCurrentLimit(CoralConstants.maxCoralCurrentAmps);

    // CoralEnc = CoralMtr.getEncoder();
   
    CoralSparkFlexConfig.softLimit.forwardSoftLimitEnabled(false);
    CoralSparkFlexConfig.softLimit.reverseSoftLimitEnabled(false);

    CoralSparkFlexConfig.encoder.positionConversionFactor(CoralConstants.outputRevPerMtrRev);
    CoralSparkFlexConfig.encoder.velocityConversionFactor(CoralConstants.outputRPMPerMtrRPM);

    // CoralController = CoralMtr.getClosedLoopController();

    // Velocity PID
    // rightCoralSparkFlexConfig.closedLoop.p(CoralConstants.kP);
    // rightCoralSparkFlexConfig.closedLoop.i(0.0);
    // rightCoralSparkFlexConfig.closedLoop.d(CoralConstants.kD);
    // CoralSparkFlexConfig.closedLoop.velocityFF(CoralConstants.feedForward);

    // MAXMotion
    // CoralSparkFlexConfig.closedLoop.maxMotion.maxAcceleration(CoralConstants.maxAccelRPMPerSec); // RPM per sec
    // CoralSparkFlexConfig.closedLoop.velocityFF(CoralConstants.feedForward);
    // rightCoralSparkFlexConfig.closedLoop.maxMotion.allowedClosedLoopError(0.0); // Rotations * positionconversionfactor
    
    CoralMtr.configure(
        CoralSparkFlexConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    backBeamBreak = new DigitalInput(CANDevices.backBeamBreakPort);
    frontBeamBreak = new DigitalInput(CANDevices.frontBeamBreakPort);
  }

  @Override
  public void periodic() {
    // CoralMtr.set(power);
    // uncomment for beam breaks
    if (isOuttaking) {
      CoralMtr.set(CoralConstants.outtakePower);
    } else if (frontBeamBreak.get() == false) {
      CoralMtr.set(0.0);
    } else if (DriverStation.isDisabled()) {
      CoralMtr.set(0.0);
    } else {
      CoralMtr.set(CoralConstants.intakePower);
    }
    SmartDashboard.putBoolean("IsOuttaking", isOuttaking);
    SmartDashboard.putBoolean("FrontBeamBreak", frontBeamBreak.get());
    SmartDashboard.putNumber("coral target power", CoralMtr.get());
  }

  // public void setPower(double power) {
  //   this.power = power;
  // }

  public void setIsOuttaking (boolean isOuttaking){
    this.isOuttaking = isOuttaking;
  }
}