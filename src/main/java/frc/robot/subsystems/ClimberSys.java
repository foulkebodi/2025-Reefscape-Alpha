package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CANDevices;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.ElevatorConstants;

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

        rightClimberMtr = new SparkMax(CANDevices.rightElevatorMtrID, MotorType.kBrushless);
        SparkMaxConfig rightClimberSparkMaxConfig = new SparkMaxConfig();

        leftClimberEnc = leftClimberMtr.getEncoder();
        rightClimberEnc = rightClimberMtr.getEncoder();
        
        leftClimberSparkMaxConfig.inverted(false);
        leftClimberSparkMaxConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
        leftClimberSparkMaxConfig.encoder.positionConversionFactor(ClimberConstants.degPerEncRev);
        leftClimberSparkMaxConfig.encoder.velocityConversionFactor(ClimberConstants.degPerSecPerRPM);

        rightClimberSparkMaxConfig.inverted(true);
        rightClimberSparkMaxConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
        rightClimberSparkMaxConfig.encoder.positionConversionFactor(ClimberConstants.degPerEncRev);
        rightClimberSparkMaxConfig.encoder.velocityConversionFactor(ElevatorConstants.inchesPerSecPerRPM);

        leftClimberSparkMaxConfig.voltageCompensation(10); 
        rightClimberSparkMaxConfig.voltageCompensation(10);
        
        leftClimberSparkMaxConfig.smartCurrentLimit(ClimberConstants.maxClimberCurrentAmps);
        rightClimberSparkMaxConfig.smartCurrentLimit(ClimberConstants.maxClimberCurrentAmps);

        leftClimberSparkMaxConfig.softLimit.forwardSoftLimitEnabled(true);
        rightClimberSparkMaxConfig.softLimit.reverseSoftLimitEnabled(true);

        leftClimberSparkMaxConfig.softLimit.forwardSoftLimit(ElevatorConstants.upperLimitInches);
        rightClimberSparkMaxConfig.softLimit.forwardSoftLimit(ElevatorConstants.upperLimitInches);
        leftClimberSparkMaxConfig.softLimit.reverseSoftLimit(ElevatorConstants.lowerLimitInches);
        rightClimberSparkMaxConfig.softLimit.reverseSoftLimit(ElevatorConstants.lowerLimitInches);

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
   
    }
        
    public double getCurrentPositionDeg() {
        return ((leftClimberEnc.getPosition() + rightClimberEnc.getPosition()) / 2);
    }

    public void setTargetDeg(double deg){
        targetDeg = deg;
        leftClimberMtr.set(climberController.calculate(leftClimberEnc.getPosition(), targetDeg));
        rightClimberMtr.set(climberController.calculate(rightClimberEnc.getPosition(), targetDeg));
    }

    public boolean isAtTarget(){
        return Math.abs(getCurrentPositionDeg() - targetDeg) < ClimberConstants.toleranceDeg;
    }
}