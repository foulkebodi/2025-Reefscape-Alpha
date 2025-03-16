package frc.robot.subsystems; 

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.CoralConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

public class IntakeSys extends SubsystemBase {
  private final SparkFlex intakeMtr;

  private final DigitalInput beamBreak;

  private boolean isOuttaking = false;

public IntakeSys() {
    intakeMtr = new SparkFlex(CANDevices.intakeMtrID, MotorType.kBrushless);
    SparkFlexConfig CoralSparkFlexConfig = new SparkFlexConfig();

    CoralSparkFlexConfig.inverted(false);

    CoralSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);

    CoralSparkFlexConfig.voltageCompensation(10);

    CoralSparkFlexConfig.smartCurrentLimit(CoralConstants.maxCoralCurrentAmps);
 
    CoralSparkFlexConfig.softLimit.forwardSoftLimitEnabled(false);
    CoralSparkFlexConfig.softLimit.reverseSoftLimitEnabled(false);

    intakeMtr.configure(
      CoralSparkFlexConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    beamBreak = new DigitalInput(CANDevices.beamBreakPort);
  }

  @Override
  public void periodic() {

  }

  public void setIsOuttaking (boolean isOuttaking){
    this.isOuttaking = isOuttaking;
  }

  public boolean getIsOuttaking (){
    return isOuttaking;
  }

  public boolean getBeamBreak() {
    return beamBreak.get();
  } 

  public double getTargetPower() {
    return intakeMtr.get();
  }
}