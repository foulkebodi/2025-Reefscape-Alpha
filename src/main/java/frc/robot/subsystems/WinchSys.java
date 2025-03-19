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
import frc.robot.Constants.WinchConstants;

public class WinchSys extends SubsystemBase {
    private final SparkMax winchMtr;

    private final RelativeEncoder winchEnc;

    private final ProfiledPIDController winchController;

    private double winchTargetDeg = 0.0;

    public WinchSys() {
        winchMtr = new SparkMax(CANDevices.winchMtrID, MotorType.kBrushless);
        SparkMaxConfig winchClimberSparkMaxConfig = new SparkMaxConfig();

        winchEnc = winchMtr.getEncoder();
        
        winchClimberSparkMaxConfig.inverted(true);
        winchClimberSparkMaxConfig.idleMode(com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake);
        winchClimberSparkMaxConfig.encoder.positionConversionFactor(WinchConstants.degPerEncRev);
        winchClimberSparkMaxConfig.encoder.velocityConversionFactor(WinchConstants.degPerSecPerRPM);
        
        winchClimberSparkMaxConfig.smartCurrentLimit(WinchConstants.maxWinchCurrentAmps);

        winchClimberSparkMaxConfig.disableVoltageCompensation();

        winchMtr.configure(
            winchClimberSparkMaxConfig,
            ResetMode.kResetSafeParameters,
            PersistMode.kPersistParameters);

        winchEnc.setPosition(0.0);

        winchController = new ProfiledPIDController(
            WinchConstants.kP, 0.0, WinchConstants.kD, 
            new Constraints(WinchConstants.maxVelDegPerSec, WinchConstants.maxAccelDegPerSecSq));
    }

    @Override
    public void periodic() {
        winchMtr.set(winchController.calculate(getWinchCurrentPositionDeg(), winchTargetDeg));
    }

    public double getWinchCurrentPositionDeg() {
        return winchEnc.getPosition();
    }

    public void setWinchTargetDeg(double deg) {
        winchTargetDeg = deg;
    }

    public double getWinchPower() {
        return winchMtr.get();
    }
}