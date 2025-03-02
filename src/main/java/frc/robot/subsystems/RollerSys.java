package frc.robot.subsystems; 

import com.revrobotics.spark.SparkFlex;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;
import frc.robot.Constants.CANDevices;


public class RollerSys extends SubsystemBase {
  private final SparkFlex rollerMtr;

  public RollerSys() {
    rollerMtr = new SparkFlex(CANDevices.rollerMtrID, MotorType.kBrushless);
    SparkFlexConfig rollerMtrSparkFlexConfig = new SparkFlexConfig();

    rollerMtrSparkFlexConfig.inverted(true);

    rollerMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);

    rollerMtrSparkFlexConfig.voltageCompensation(10);

    rollerMtrSparkFlexConfig.smartCurrentLimit(RollerConstants.maxRollerCurrentAmps);

    rollerMtr.configure(
        rollerMtrSparkFlexConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {

  }

  public void setPower(double power) {
    rollerMtr.set(power);
  }
}