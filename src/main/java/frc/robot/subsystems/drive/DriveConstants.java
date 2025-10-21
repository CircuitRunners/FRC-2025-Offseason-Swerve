package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class DriveConstants {
    public static final double kDriveMaxAngularRate = 8.2;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 20.0;
    public static final double kDriveMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double kMaxAccelerationMetersPerSecondSquared = 10.0;
    public static final double kSteerJoystickDeadband = 0.05;
    public static final double kHeadingControllerP = 5.0;
    public static final double kHeadingControllerI = 0;
    public static final double kHeadingControllerD = 0;

    public static final Translation2d kTranslation2dZero = new Translation2d(0.0, 0.0);
    public static final Rotation2d kRotation2dZero = new Rotation2d();

    public static final RobotConfig robotConfig;
    static {
        RobotConfig tempConfig = null;
        try{
            tempConfig = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }
        robotConfig = tempConfig;
    }
}
