package frc.robot.subsystems; 

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.IntakeConstants;

public class IntakeSys extends SubsystemBase {
  private final SparkFlex intakeMtr;
  private final DigitalInput beamBreak;
  private final Debouncer currentDebouncer;
  private final Debouncer beamBreakDebouncer;
  private final MedianFilter filter;

  private boolean intaking = false;
  private boolean outtaking = false;
  private double startTime = 0.0;

public IntakeSys() {
    intakeMtr = new SparkFlex(CANDevices.intakeMtrID, MotorType.kBrushless);
    SparkFlexConfig intakeSparkFlexConfig = new SparkFlexConfig();

    intakeSparkFlexConfig.inverted(true);

    intakeSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);

    intakeSparkFlexConfig.smartCurrentLimit(IntakeConstants.maxIntakeCurrentAmps);

    intakeSparkFlexConfig.voltageCompensation(9);

    intakeSparkFlexConfig.softLimit.forwardSoftLimitEnabled(false);
    intakeSparkFlexConfig.softLimit.reverseSoftLimitEnabled(false);

    intakeMtr.configure(
      intakeSparkFlexConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    beamBreak = new DigitalInput(CANDevices.beamBreakPort);

    currentDebouncer = new Debouncer(IntakeConstants.currentDebounceTime, DebounceType.kRising);
    beamBreakDebouncer = new Debouncer(IntakeConstants.beamBreakDebounceTime, DebounceType.kBoth);

    filter = new MedianFilter(IntakeConstants.filterSize);
  }

  @Override
  public void periodic() {
    if(outtaking) {
      intakeMtr.set(IntakeConstants.outtakePower);
    } else if(intaking && !getFilteredBeamBreak()) {
      intakeMtr.set(IntakeConstants.idleOutPower);
      startTime = System.currentTimeMillis();
    } else if(intaking && getFilteredBeamBreak()) {
      if((System.currentTimeMillis() - startTime) > (IntakeConstants.waitSeconds * 1000.0)) {
        intakeMtr.set(IntakeConstants.intakePower);
        if(currentDebouncer.calculate(getFilteredOutputCurrent() >= IntakeConstants.currentThreshold)) {
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

  public boolean getFilteredBeamBreak() {
    return beamBreakDebouncer.calculate(beamBreak.get());
  }

  public double getTargetPower() {
    return intakeMtr.get();
  }

  public double getFilteredOutputCurrent() {
    return filter.calculate(intakeMtr.getOutputCurrent());
  }

  public double getIntakeCurrentTimeMillis() {
    return System.currentTimeMillis() - startTime;
  }

  public boolean getIntaking() {
    return intaking;
  }   
  public boolean getOuttaking() {
    return outtaking;
  } 
}