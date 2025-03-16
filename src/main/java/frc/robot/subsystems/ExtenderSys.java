package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ExtenderConstants;

public class ExtenderSys extends SubsystemBase {
    private final SparkFlex extenderMtr;

    private final RelativeEncoder extenderSys;

    private final ProfiledPIDController extenderController;

    private double targetInches = 0.0;

    public ExtenderSys() {
        extenderMtr = new SparkFlex(CANDevices.leftElevatorMtrID, MotorType.kBrushless);
        SparkFlexConfig extenderMtrSparkFlexConfig = new SparkFlexConfig();

        extenderSys = extenderMtr.getEncoder();
        
        extenderMtrSparkFlexConfig.inverted(false);
        extenderMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
        extenderMtrSparkFlexConfig.encoder.positionConversionFactor(ExtenderConstants.inchesPerMtrRev);
        extenderMtrSparkFlexConfig.encoder.velocityConversionFactor(ExtenderConstants.inchesPerSecPerRPM);

        extenderMtrSparkFlexConfig.voltageCompensation(11);
        
        extenderMtrSparkFlexConfig.smartCurrentLimit(ExtenderConstants.maxExtenderCurrentAmps);

        extenderMtrSparkFlexConfig.softLimit.forwardSoftLimitEnabled(true);

        extenderMtrSparkFlexConfig.softLimit.forwardSoftLimit(ExtenderConstants.upperLimitInches);
        extenderMtrSparkFlexConfig.softLimit.reverseSoftLimit(ExtenderConstants.lowerLimitInches);

        extenderMtr.configure(
            extenderMtrSparkFlexConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        extenderSys.setPosition(ExtenderConstants.homePresetInches);

        extenderController = new ProfiledPIDController(
        ExtenderConstants.kP, 0.0, ExtenderConstants.kD, 
        new Constraints(ExtenderConstants.maxVelInchesPerSec, ExtenderConstants.maxAccelInchesPerSecSq));
    }

    @Override
    public void periodic(){
            extenderMtr.set(extenderController.calculate(extenderSys.getPosition(), targetInches));
        if(DriverStation.isDisabled()){
            targetInches = getCurrentPositionInches();
            extenderController.reset(targetInches);
        }
    }

    public double getCurrentPositionInches() {
        return extenderSys.getPosition();
    }

    public void setTargetInches(double inches){
        targetInches = inches;
    }

    public double getTargetInches(){
        return targetInches;
    }

    public boolean isAtTarget(){
        return Math.abs(getCurrentPositionInches() - targetInches) < ExtenderConstants.toleranceInches;
    }

    public double getErrorInches(){
        return Math.abs(getCurrentPositionInches() - targetInches);
    }
}