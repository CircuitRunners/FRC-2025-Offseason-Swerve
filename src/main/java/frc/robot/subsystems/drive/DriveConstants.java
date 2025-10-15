package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

public class DriveConstants {
    public static final double kDriveMaxAngularRate = 8.2;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 20.0;
    public static final double kDriveMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double kMaxAccelerationMetersPerSecondSquared = 10.0;
}
