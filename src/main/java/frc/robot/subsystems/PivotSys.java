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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.CANDevices;

public class PivotSys extends SubsystemBase {
    private final SparkFlex pivotMtr;

    private final ProfiledPIDController pivotController;

    private final DutyCycleEncoder absPivotEnc;

    private double targetDeg = 0.0;

    private double manualPower = 0.0;
    
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

        pivotController = new ProfiledPIDController(
        PivotConstants.kP, 0.0, PivotConstants.kD, 
        new Constraints(PivotConstants.maxVelDegPerSec, PivotConstants.maxAccelDegPerSecSq));
    }
 
    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        if(manualPower == 0.0) {
            pivotMtr.set(pivotController.calculate(getCurrentPositionDeg(), targetDeg));
        }
        else {
            pivotMtr.set(manualPower);
            targetDeg = getCurrentPositionDeg();
            pivotController.reset(targetDeg);
        }
        if(DriverStation.isDisabled()) {
            targetDeg = getCurrentPositionDeg();
            pivotController.reset(targetDeg);
        }
        SmartDashboard.putNumber("pivot error deg", getCurrentPositionDeg() - targetDeg);
    }

    // Put methods for controlling this subsystem here. Call these from Commands.
    public double getCurrentPositionDeg() {
            return (absPivotEnc.get());
    }

    public void setTargetDeg(double degrees) {
        targetDeg = degrees;
    }

    public void setManualSpeedDegPerSec(double degPerSec) {
        double manualPower = (degPerSec / 6.0) / PivotConstants.freeSpeedRPM;
        this.manualPower = manualPower;
    }

    public boolean isAtTarget() {
        return Math.abs(getCurrentPositionDeg() - targetDeg) < PivotConstants.toleranceDeg;
    }
}