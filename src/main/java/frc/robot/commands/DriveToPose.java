package frc.robot.commands;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.auto.AutoConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;

public class DriveToPose extends Command{
    private ProfiledPIDController driveController;


    private final ProfiledPIDController thetaController =
            new ProfiledPIDController(
                    AutoConstants.kPThetaController,
                    0.0,
                    0.0,
                    new TrapezoidProfile.Constraints(
                            DriveConstants.kDriveMaxAngularRate,
                            DriveConstants.kMaxAngularSpeedRadiansPerSecondSquared),
                    0.02);
    

    private Drive drive;
    private double driveErrorAbs;
    private double thetaErrorAbs;
    private double ffMinRadius = 0.0, ffMaxRadius = 0.1;
    private Pose2d targetLocation;

    public DriveToPose (
        Drive drive, 
        Pose2d targetLocation, 
        double constraintFactor) {
            this.drive = drive;
            this.targetLocation = targetLocation;
            this.driveController = 
                new ProfiledPIDController(
                            AutoConstants.kPXYController,
                            0.0,
                            0.0,
                            new TrapezoidProfile.Constraints(
                                    DriveConstants.kDriveMaxSpeed * constraintFactor,
                                    DriveConstants.kMaxAccelerationMetersPerSecondSquared
                                            * constraintFactor),
                            0.02);
            addRequirements(drive);
            thetaController.enableContinuousInput(-Math.PI, Math.PI);
        }

    @Override
    public void initialize() {
        Pose2d currentPose = drive.getPose();

        driveController.reset(
            currentPose.getTranslation().getDistance(targetLocation.getTranslation()),
             Math.min(
                        0.0,
                        -new Translation2d(
                                        drive.getFieldRelativeChassisSpeeds()
                                                .vxMetersPerSecond,
                                        drive.getFieldRelativeChassisSpeeds()
                                                .vyMetersPerSecond)
                                .rotateBy(
                                        targetLocation
                                                .getTranslation()
                                                .minus(
                                                        drive
                                                                .getPose()
                                                                .getTranslation())
                                                .getAngle()
                                                .unaryMinus())
                                .getX()));
        
    }
}

