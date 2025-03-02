package frc.robot.subsystems; 

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.CoralConstants;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;


public class CoralSys extends SubsystemBase {
  private final SparkFlex CoralMtr;

  private final DigitalInput backBeamBreak;
  private final DigitalInput frontBeamBreak;

  private boolean isOuttaking = false;

public CoralSys() {
    CoralMtr = new SparkFlex(CANDevices.CoralMtrID, MotorType.kBrushless);
    SparkFlexConfig CoralSparkFlexConfig = new SparkFlexConfig();

    CoralSparkFlexConfig.inverted(true);

    CoralSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);

    CoralSparkFlexConfig.voltageCompensation(10);

    CoralSparkFlexConfig.smartCurrentLimit(CoralConstants.maxCoralCurrentAmps);
   
    CoralSparkFlexConfig.softLimit.forwardSoftLimitEnabled(false);
    CoralSparkFlexConfig.softLimit.reverseSoftLimitEnabled(false);

    CoralMtr.configure(
      CoralSparkFlexConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    backBeamBreak = new DigitalInput(CANDevices.backBeamBreakPort);
    frontBeamBreak = new DigitalInput(CANDevices.frontBeamBreakPort);
  }

  @Override
  public void periodic() {
    if (isOuttaking) {
      CoralMtr.set(CoralConstants.outtakePower);
    } else if (getFrontBeamBreak() == false && getBackBeamBreak() == true) {
      CoralMtr.set(0.0);
    } else if (DriverStation.isDisabled()) {
      CoralMtr.set(0.0);
    }  else {
      CoralMtr.set(CoralConstants.intakePower);
   }
  }

  public void setIsOuttaking (boolean isOuttaking){
    this.isOuttaking = isOuttaking;
  }

  public boolean getIsOuttaking (){
    return isOuttaking;
  }

  public boolean getFrontBeamBreak() {
    return frontBeamBreak.get();
  } 

  public boolean getBackBeamBreak() {
    return backBeamBreak.get();
  }

  public double getTargetPower() {
    return CoralMtr.get();
  }
  
}