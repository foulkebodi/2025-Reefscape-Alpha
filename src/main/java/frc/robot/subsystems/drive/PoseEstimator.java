package frc.robot.subsystems.drive;

import java.util.function.Supplier;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.util.LimelightHelpers;

public class PoseEstimator extends SubsystemBase {

    private final SwerveDrivePoseEstimator poseEstimator;

    private final Supplier<Rotation2d> gyroHeadingSupplier;
    private final Supplier<SwerveModulePosition[]> modulePositionsSupplier;
    

    public PoseEstimator(
        SwerveDriveKinematics swerveKinematics,
        Supplier<Rotation2d> gyroHeadingSupplier,
        Supplier<SwerveModulePosition[]> modulePositionsSupplier) {

        poseEstimator = new SwerveDrivePoseEstimator(
            swerveKinematics,
            gyroHeadingSupplier.get(),
            modulePositionsSupplier.get(),
            new Pose2d(),
            VecBuilder.fill(
                VisionConstants.odomTranslationStdDevMeters,
                VisionConstants.odomTranslationStdDevMeters,
                VisionConstants.odomRotationStdDevRad),
            VecBuilder.fill(
                VisionConstants.visionTranslationStdDevMeters,
                VisionConstants.visionTranslationStdDevMeters,
                VisionConstants.visionRotationStdDevRad));

        this.gyroHeadingSupplier = gyroHeadingSupplier;
        this.modulePositionsSupplier = modulePositionsSupplier;
    }

    @Override
    public void periodic() {
        if (LimelightHelpers.getTV(VisionConstants.frontLimelightName)
        // && Math.abs(getBackLimelightPose().getX() - get().getX()) < VisionConstants.poseInnacuracyThreshold
        /* && Math.abs(getBackLimelightPose().getY() - get().getY()) < VisionConstants.poseInnacuracyThreshold */) {
            poseEstimator.addVisionMeasurement(getFrontLimelightPose(), getFrontLimelightTimestamp());
        }
        if (LimelightHelpers.getTV(VisionConstants.backLimelightName) 
        // && Math.abs(getBackLimelightPose().getX() - get().getX()) < VisionConstants.poseInnacuracyThreshold
        /*&& Math.abs(getBackLimelightPose().getY() - get().getY()) < VisionConstants.poseInnacuracyThreshold */) {
            poseEstimator.addVisionMeasurement(getBackLimelightPose(), getBackLimelightTimestamp());
        }
        poseEstimator.update(gyroHeadingSupplier.get(), modulePositionsSupplier.get()); 
    }

    public Pose2d get() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(gyroHeadingSupplier.get(), modulePositionsSupplier.get(), pose);
    }   

    public void resetHeading() {
        poseEstimator.resetPosition(gyroHeadingSupplier.get(), modulePositionsSupplier.get(), 
        new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), Rotation2d.fromDegrees(0)));
        // DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red ? Rotation2d.fromDegrees(0) : Rotation2d.fromDegrees(180)));
    }

    public Pose2d getFrontLimelightPose() {
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.frontLimelightName);
        return limelightMeasurement.pose;
    }

    public double getFrontLimelightTimestamp() {
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.frontLimelightName);
        return limelightMeasurement.timestampSeconds;
    }    

    public Pose2d getBackLimelightPose() {
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.backLimelightName);
        return limelightMeasurement.pose;
    }

    public double getBackLimelightTimestamp() {
        LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.backLimelightName);
        return limelightMeasurement.timestampSeconds;
    }
}