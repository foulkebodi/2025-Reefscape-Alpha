package frc.robot.subsystems; 
 
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.CANDevices;

public class PivotSys extends SubsystemBase {
    private final SparkFlex pivotMtr;

    private final ProfiledPIDController pivotController;

    private final DutyCycleEncoder absPivotEnc;

    private double targetDeg = 0.0;
    
    public PivotSys() {
        pivotMtr = new SparkFlex(CANDevices.pivotMtrID, MotorType.kBrushless);
        SparkFlexConfig pivotMtrSparkFlexConfig = new SparkFlexConfig();

        pivotMtrSparkFlexConfig.inverted(false);

        pivotMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);

        pivotMtrSparkFlexConfig.voltageCompensation(10);

        pivotMtrSparkFlexConfig.smartCurrentLimit(PivotConstants.maxPivotCurrentAmps);

        pivotMtrSparkFlexConfig.softLimit.forwardSoftLimitEnabled(true);
        pivotMtrSparkFlexConfig.softLimit.reverseSoftLimitEnabled(true);
        pivotMtrSparkFlexConfig.softLimit.forwardSoftLimit(PivotConstants.upperLimitDeg);
        pivotMtrSparkFlexConfig.softLimit.reverseSoftLimit(PivotConstants.lowerLimitDeg);

        pivotMtrSparkFlexConfig.encoder.positionConversionFactor(PivotConstants.degPerEncRev);
        pivotMtrSparkFlexConfig.encoder.velocityConversionFactor(PivotConstants.degPerSecPerRPM);

        pivotMtr.configure(
            pivotMtrSparkFlexConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        absPivotEnc = new DutyCycleEncoder(CANDevices.pivotEncPortID, 360.0, PivotConstants.absPivotEncOffsetDeg);
        absPivotEnc.setInverted(false);

        pivotController = new ProfiledPIDController(
        PivotConstants.kP, 0.0, PivotConstants.kD, 
        new Constraints(PivotConstants.maxVelDegPerSec, PivotConstants.maxAccelDegPerSecSq));
    }
 
    @Override
    public void periodic() {
        pivotMtr.set(pivotController.calculate(getCurrentPositionDeg(), targetDeg));
        if(DriverStation.isDisabled()){
            targetDeg = getCurrentPositionDeg();
            pivotController.reset(targetDeg);
        }
    }

    public double getCurrentPositionDeg() {
        return absPivotEnc.get();
    }

    public void setTargetDeg(double degrees) {
        targetDeg = degrees;
    }

    public double getTargetDeg() {
        return targetDeg;
    }

    public boolean isAtTarget() {
        return Math.abs(getCurrentPositionDeg() - targetDeg) < PivotConstants.toleranceDeg;
    }

    public double getErrorDeg(){
        return Math.abs(getCurrentPositionDeg() - targetDeg);
    }
}