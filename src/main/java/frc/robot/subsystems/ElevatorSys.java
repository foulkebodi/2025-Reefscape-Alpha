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
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSys extends SubsystemBase {
    private final SparkFlex leftElevatorMtr;
    private final SparkFlex rightElevatorMtr;

    private final RelativeEncoder leftElevatorEnc;
    private final RelativeEncoder rightElevatorEnc;

    private final ProfiledPIDController elevatorController;

    private double targetInches = 0.0;
    private double manualPower = 0.0;

    public ElevatorSys() {
        leftElevatorMtr = new SparkFlex(CANDevices.leftElevatorMtrID, MotorType.kBrushless);
        SparkFlexConfig leftElevatorMtrSparkFlexConfig = new SparkFlexConfig();

        rightElevatorMtr = new SparkFlex(CANDevices.rightElevatorMtrID, MotorType.kBrushless);
        SparkFlexConfig rightElevatorMtrSparkFlexConfig = new SparkFlexConfig();

        leftElevatorEnc = leftElevatorMtr.getEncoder();
        rightElevatorEnc = rightElevatorMtr.getEncoder();
        
        leftElevatorMtrSparkFlexConfig.inverted(true);
        leftElevatorMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
        leftElevatorMtrSparkFlexConfig.encoder.positionConversionFactor(ElevatorConstants.inchesPerMtrRev);
        leftElevatorMtrSparkFlexConfig.encoder.velocityConversionFactor(ElevatorConstants.inchesPerSecPerRPM);

        rightElevatorMtrSparkFlexConfig.inverted(false);
        rightElevatorMtrSparkFlexConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
        rightElevatorMtrSparkFlexConfig.encoder.positionConversionFactor(ElevatorConstants.inchesPerMtrRev);
        rightElevatorMtrSparkFlexConfig.encoder.velocityConversionFactor(ElevatorConstants.inchesPerSecPerRPM);

        leftElevatorMtrSparkFlexConfig.voltageCompensation(11);
        rightElevatorMtrSparkFlexConfig.voltageCompensation(11);
        
        leftElevatorMtrSparkFlexConfig.smartCurrentLimit(ElevatorConstants.maxElevatorCurrentAmps);
        rightElevatorMtrSparkFlexConfig.smartCurrentLimit(ElevatorConstants.maxElevatorCurrentAmps);

        leftElevatorMtrSparkFlexConfig.softLimit.forwardSoftLimitEnabled(true);
        rightElevatorMtrSparkFlexConfig.softLimit.reverseSoftLimitEnabled(true);

        leftElevatorMtrSparkFlexConfig.softLimit.forwardSoftLimit(ElevatorConstants.upperLimitInches);
        rightElevatorMtrSparkFlexConfig.softLimit.forwardSoftLimit(ElevatorConstants.upperLimitInches);
        leftElevatorMtrSparkFlexConfig.softLimit.reverseSoftLimit(ElevatorConstants.lowerLimitInches);
        rightElevatorMtrSparkFlexConfig.softLimit.reverseSoftLimit(ElevatorConstants.lowerLimitInches);

        leftElevatorMtr.configure(
            leftElevatorMtrSparkFlexConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        rightElevatorMtr.configure(
            rightElevatorMtrSparkFlexConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        leftElevatorEnc.setPosition(ElevatorConstants.homeInches);
        rightElevatorEnc.setPosition(ElevatorConstants.homeInches);

        elevatorController = new ProfiledPIDController(
        ElevatorConstants.kP, 0.0, ElevatorConstants.kD, 
        new Constraints(ElevatorConstants.maxVelInchesPerSec, ElevatorConstants.maxAccelInchesPerSecSq));
    }

    @Override
    public void periodic(){
        if(manualPower == 0.0){
            leftElevatorMtr.set(elevatorController.calculate(leftElevatorEnc.getPosition(),targetInches));
            rightElevatorMtr.set(elevatorController.calculate(rightElevatorEnc.getPosition(),targetInches));
        }
        else {
            leftElevatorMtr.set(manualPower);
            rightElevatorMtr.set(manualPower);
            targetInches = getCurrentPositionInches();
            elevatorController.reset(targetInches);
        }
        if(DriverStation.isDisabled()){
            targetInches = getCurrentPositionInches();
            elevatorController.reset(targetInches);
        }
    }

    public double getCurrentPositionInches() {
        return ((leftElevatorEnc.getPosition() + rightElevatorEnc.getPosition()) / 2);
    }
    
    public double getLeftCurrentPositionInches() {
        return leftElevatorEnc.getPosition();
    }

    public double getRightCurrentPositionInches() {
        return rightElevatorEnc.getPosition();
    }

    public void setTargetInches(double inches){
        targetInches = inches;
    }

    public double getTargetInches(){
        return targetInches;
    }

    public void setManualSpeedInchesPerSec (double inchesPersec){
        double manualPower = inchesPersec / ElevatorConstants.freeSpeedInchesPerSec;
        this.manualPower = manualPower;
    }

    public boolean isAtTarget(){
        return Math.abs(getCurrentPositionInches() - targetInches) < ElevatorConstants.toleranceInches;
    }

    public double getErrorInches(){
        return Math.abs(getCurrentPositionInches() - targetInches);
    }
}