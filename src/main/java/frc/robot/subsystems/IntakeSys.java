package frc.robot.subsystems; 

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.IntakeConstants;

public class IntakeSys extends SubsystemBase {
  private final SparkFlex intakeMtr;
  private final DigitalInput beamBreak;

  private boolean intaking = false;
  private boolean outtaking = false;
  private double startTime = 0.0;

public IntakeSys() {
    intakeMtr = new SparkFlex(CANDevices.intakeMtrID, MotorType.kBrushless);
    SparkFlexConfig intakeSparkFlexConfig = new SparkFlexConfig();

    intakeSparkFlexConfig.inverted(true);

    intakeSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);

    intakeSparkFlexConfig.smartCurrentLimit(IntakeConstants.maxIntakeCurrentAmps);
 
    intakeSparkFlexConfig.softLimit.forwardSoftLimitEnabled(false);
    intakeSparkFlexConfig.softLimit.reverseSoftLimitEnabled(false);

    intakeMtr.configure(
      intakeSparkFlexConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    beamBreak = new DigitalInput(CANDevices.beamBreakPort);
  }

  @Override
  public void periodic() {
    if(outtaking) {
      intakeMtr.set(IntakeConstants.outtakePower);
    } else if(intaking && !getBeamBreak()) {
      intakeMtr.set(IntakeConstants.idleOutPower);
      startTime = System.currentTimeMillis();
    } else if(intaking && getBeamBreak()) {
      if((System.currentTimeMillis() - startTime) > (IntakeConstants.WaitSeconds * 1000.0)) {
        intakeMtr.set(IntakeConstants.intakePower);
        if(intakeMtr.getOutputCurrent() > IntakeConstants.CurrentThreshold) {
          intaking = false;
        }
      }
    } else {
      intakeMtr.set(IntakeConstants.idlePower);
    }
  }

  public void setIsIntaking(boolean isIntaking) {
    intaking = isIntaking;
  }

  public void setIsOuttaking(boolean isOuttaking) {
    outtaking = isOuttaking;
  }

  public boolean getBeamBreak() {
    return beamBreak.get();
  } 

  public double getTargetPower() {
    return intakeMtr.get();
  }

  public double getOutputCurrent() {
    return intakeMtr.getOutputCurrent();
  }

  public double getCurrentTimeMillis() {
    return System.currentTimeMillis() - startTime;
  }

  public boolean getIntaking() {
    return intaking;
  }   
  public boolean getOuttaking() {
    return outtaking;
  } 
}