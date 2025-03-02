package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ClimberConstants;

public class ClimberSys extends SubsystemBase {
    private final SparkMax winchClimberMtr;
    private final SparkMax rightClimberMtr;

    private final RelativeEncoder winchClimberEnc;
    private final RelativeEncoder rightClimberEnc;

    private final ProfiledPIDController climberController;

    private double climberTargetDeg = 0.0;

    private double winchTargetDeg = 0.0;

    public ClimberSys() {
        winchClimberMtr = new SparkMax(CANDevices.leftClimberMtrID, MotorType.kBrushless);
        SparkMaxConfig winchClimberSparkMaxConfig = new SparkMaxConfig();

        rightClimberMtr = new SparkMax(CANDevices.rightClimberMtrID, MotorType.kBrushless);
        SparkMaxConfig rightClimberSparkMaxConfig = new SparkMaxConfig();

        winchClimberEnc = winchClimberMtr.getEncoder();
        rightClimberEnc = rightClimberMtr.getEncoder();
        
        winchClimberSparkMaxConfig.inverted(false);
        winchClimberSparkMaxConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
        winchClimberSparkMaxConfig.encoder.positionConversionFactor(ClimberConstants.degPerEncRev);
        winchClimberSparkMaxConfig.encoder.velocityConversionFactor(ClimberConstants.degPerSecPerRPM);

        rightClimberSparkMaxConfig.inverted(false);
        rightClimberSparkMaxConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
        rightClimberSparkMaxConfig.encoder.positionConversionFactor(ClimberConstants.degPerEncRev);
        rightClimberSparkMaxConfig.encoder.velocityConversionFactor(ClimberConstants.degPerSecPerRPM);
        
        winchClimberSparkMaxConfig.smartCurrentLimit(ClimberConstants.maxClimberCurrentAmps);
        rightClimberSparkMaxConfig.smartCurrentLimit(ClimberConstants.maxClimberCurrentAmps);

        winchClimberMtr.configure(
            winchClimberSparkMaxConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        rightClimberMtr.configure(
            rightClimberSparkMaxConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        winchClimberEnc.setPosition(ClimberConstants.homePresetDeg);
        rightClimberEnc.setPosition(ClimberConstants.homePresetDeg);

        climberController = new ProfiledPIDController(
        ClimberConstants.kP, 0.0, ClimberConstants.kD, 
        new Constraints(ClimberConstants.maxVelDegPerSec, ClimberConstants.maxAccelDegPerSecSq));
    }

    @Override
    public void periodic() {
        winchClimberMtr.set(climberController.calculate(getWinchCurrentPositionDeg(), winchTargetDeg));
        rightClimberMtr.set(climberController.calculate(getRightCurrentPositionDeg(), climberTargetDeg));
    }

    public double getWinchCurrentPositionDeg() {
        return winchClimberEnc.getPosition();
    }
    
    public double getRightCurrentPositionDeg() {
        return rightClimberEnc.getPosition();
    }

    public void setClimberTargetDeg(double deg) {
        climberTargetDeg = deg;
    }

    public void setWinchTargetDeg(double deg) {
        winchTargetDeg = deg;
    }

    public double getClimberTargetDeg() {
        return climberTargetDeg;
    }

    public double getWinchPower() {
        return winchClimberMtr.get();
    }

    public double getRightPower() {
        return rightClimberMtr.get();
    }
}