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
    private final SparkMax leftClimberMtr;
    private final SparkMax rightClimberMtr;

    private final RelativeEncoder leftClimberEnc;
    private final RelativeEncoder rightClimberEnc;

    private final ProfiledPIDController climberController;

    private double targetDeg = 0.0;


    public ClimberSys() {
        leftClimberMtr = new SparkMax(CANDevices.leftClimberMtrID, MotorType.kBrushless);
        SparkMaxConfig leftClimberSparkMaxConfig = new SparkMaxConfig();

        rightClimberMtr = new SparkMax(CANDevices.rightClimberMtrID, MotorType.kBrushless);
        SparkMaxConfig rightClimberSparkMaxConfig = new SparkMaxConfig();

        leftClimberEnc = leftClimberMtr.getEncoder();
        rightClimberEnc = rightClimberMtr.getEncoder();
        
        leftClimberSparkMaxConfig.inverted(true);

        leftClimberSparkMaxConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
        leftClimberSparkMaxConfig.encoder.positionConversionFactor(ClimberConstants.degPerEncRev);
        leftClimberSparkMaxConfig.encoder.velocityConversionFactor(ClimberConstants.degPerSecPerRPM);

        rightClimberSparkMaxConfig.inverted(false);
        rightClimberSparkMaxConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
        rightClimberSparkMaxConfig.encoder.positionConversionFactor(ClimberConstants.degPerEncRev);
        rightClimberSparkMaxConfig.encoder.velocityConversionFactor(ClimberConstants.degPerSecPerRPM);
        
        leftClimberSparkMaxConfig.smartCurrentLimit(ClimberConstants.maxClimberCurrentAmps);
        rightClimberSparkMaxConfig.smartCurrentLimit(ClimberConstants.maxClimberCurrentAmps);

        leftClimberMtr.configure(
            leftClimberSparkMaxConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        rightClimberMtr.configure(
            rightClimberSparkMaxConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        leftClimberEnc.setPosition(ClimberConstants.homePresetDeg);
        rightClimberEnc.setPosition(ClimberConstants.homePresetDeg);

        climberController = new ProfiledPIDController(
        ClimberConstants.kP, 0.0, ClimberConstants.kD, 
        new Constraints(ClimberConstants.maxVelDegPerSec, ClimberConstants.maxAccelDegPerSecSq));
    }

    @Override
    public void periodic() {
        leftClimberMtr.set(climberController.calculate(getLeftCurrentPositionDeg(), targetDeg));
        rightClimberMtr.set(climberController.calculate(getLeftCurrentPositionDeg(), targetDeg));
    }
        
    public double getCurrentPositionDeg() {
        return ((leftClimberEnc.getPosition() + rightClimberEnc.getPosition()) / 2);
    }

    public double getLeftCurrentPositionDeg() {
        return leftClimberEnc.getPosition();
    }
    
    public double getRightCurrentPositionDeg() {
        return rightClimberEnc.getPosition();
    }

    public void setTargetDeg(double deg) {
        targetDeg = deg;
    }

    public boolean isAtTarget() {
        return Math.abs(getCurrentPositionDeg() - targetDeg) < ClimberConstants.toleranceDeg;
    }

    public double getTargetDeg() {
        return targetDeg;
    }

    public double getLeftPower() {
        return leftClimberMtr.get();
    }

    public double getRightPower() {
        return rightClimberMtr.get();
    }
}