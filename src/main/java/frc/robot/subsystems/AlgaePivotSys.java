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

import frc.robot.Constants.AlgaePivotConstants;
import frc.robot.Constants.CANDevices;

public class AlgaePivotSys extends SubsystemBase {
    private final SparkFlex algaePivotMtr;

    private final ProfiledPIDController pivotController;

    private final DutyCycleEncoder absPivotEnc;

    private double targetDeg = 0.0;

    private double manualPower = 0.0;
    
    public AlgaePivotSys() {

        algaePivotMtr = new SparkFlex(CANDevices.algaePivotMtrID, MotorType.kBrushless);
        SparkFlexConfig algaePivotMtrSparkFlexConfig = new SparkFlexConfig();

        algaePivotMtrSparkFlexConfig.inverted(false);

        algaePivotMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);

        algaePivotMtrSparkFlexConfig.voltageCompensation(10);

        algaePivotMtrSparkFlexConfig.smartCurrentLimit(AlgaePivotConstants.maxPivotCurrentAmps);

        algaePivotMtrSparkFlexConfig.softLimit.forwardSoftLimitEnabled(true);
        algaePivotMtrSparkFlexConfig.softLimit.reverseSoftLimitEnabled(true);
        algaePivotMtrSparkFlexConfig.softLimit.forwardSoftLimit(AlgaePivotConstants.upperLimitDeg);
        algaePivotMtrSparkFlexConfig.softLimit.reverseSoftLimit(AlgaePivotConstants.lowerLimitDeg);

        algaePivotMtrSparkFlexConfig.encoder.positionConversionFactor(AlgaePivotConstants.degPerEncRev);
        algaePivotMtrSparkFlexConfig.encoder.velocityConversionFactor(AlgaePivotConstants.degPerSecPerRPM);

        algaePivotMtr.configure(
            algaePivotMtrSparkFlexConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        absPivotEnc = new DutyCycleEncoder(CANDevices.algaePivotEncPort, 360.0, AlgaePivotConstants.absPivotEncOffsetDeg);

        pivotController = new ProfiledPIDController(
        AlgaePivotConstants.kP, 0.0, AlgaePivotConstants.kD, 
        new Constraints(AlgaePivotConstants.maxVelDegPerSec, AlgaePivotConstants.maxAccelDegPerSecSq));
    }
 
    // This method will be called once per scheduler run
    @Override
    public void periodic() {
        if(manualPower == 0.0) {
            algaePivotMtr.set(pivotController.calculate(getCurrentPositionDeg(), targetDeg));
        }
        else {
            algaePivotMtr.set(manualPower);
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
        double manualPower = (degPerSec / 6.0) / AlgaePivotConstants.freeSpeedRPM;
        this.manualPower = manualPower;
    }

    public boolean isAtTarget() {
        return Math.abs(getCurrentPositionDeg() - targetDeg) < AlgaePivotConstants.toleranceDeg;
    }
}