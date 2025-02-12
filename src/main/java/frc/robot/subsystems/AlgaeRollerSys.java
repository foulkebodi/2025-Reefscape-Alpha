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
import frc.robot.Constants.AlgaePivotConstants;
import frc.robot.Constants.CANDevices;


public class AlgaeRollerSys extends SubsystemBase {
  private final SparkFlex algaeRollerMtr;

  private final RelativeEncoder algaeRollerEnc;

  private final SparkClosedLoopController alageRollerController;

  public AlgaeRollerSys() {
    algaeRollerMtr = new SparkFlex(CANDevices.algaeRollerMtrID, MotorType.kBrushless);
    SparkFlexConfig algaeRollerMtrSparkFlexConfig = new SparkFlexConfig();

    algaeRollerMtrSparkFlexConfig.inverted(true);

    algaeRollerMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);

    algaeRollerMtrSparkFlexConfig.voltageCompensation(10);

    algaeRollerMtrSparkFlexConfig.smartCurrentLimit(AlgaePivotConstants.maxPivotCurrentAmps);

    algaeRollerEnc = algaeRollerMtr.getEncoder();

    algaeRollerMtrSparkFlexConfig.encoder.positionConversionFactor(0.0);
    algaeRollerMtrSparkFlexConfig.encoder.velocityConversionFactor(0.0);

    alageRollerController = algaeRollerMtr.getClosedLoopController();

    algaeRollerMtrSparkFlexConfig.closedLoop.p(0.0);
    algaeRollerMtrSparkFlexConfig.closedLoop.i(0.0);
    algaeRollerMtrSparkFlexConfig.closedLoop.d(0.0);
    algaeRollerMtrSparkFlexConfig.closedLoop.velocityFF(0.0);
    algaeRollerMtrSparkFlexConfig.closedLoop.maxMotion.maxAcceleration(0.0); // RPM per sec
    algaeRollerMtrSparkFlexConfig.closedLoop.maxMotion.allowedClosedLoopError(0.0); // Rotations * positionconversionfactor


    algaeRollerMtr.configure(
        algaeRollerMtrSparkFlexConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
      
  }

  public void setRPM(double rpm) {
    alageRollerController.setReference(rpm, ControlType.kVelocity);
    alageRollerController.setReference(rpm, ControlType.kVelocity);
  }

  public double getRPM() {
    return algaeRollerMtr.getEncoder().getVelocity();
}
}