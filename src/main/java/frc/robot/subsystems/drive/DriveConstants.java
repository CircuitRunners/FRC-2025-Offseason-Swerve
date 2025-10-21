package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import frc.lib.util.SynchronousPIDF;

public class DriveConstants {
    public static final double kDriveMaxAngularRate = 8.2; // 254
    public static final AngularVelocity kMaxAngularRate = Units.RadiansPerSecond.of(2.75 * Math.PI); // 1678
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = 20.0;
    public static final double kDriveMaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // 254
    public static final LinearVelocity kMaxSpeed = TunerConstants.kSpeedAt12Volts; // 1678
    public static final double kMaxAccelerationMetersPerSecondSquared = 10.0;
    public static final Translation2d kTranslation2dZero = new Translation2d(0.0, 0.0);
    public static final Rotation2d kRotation2dZero = new Rotation2d();

    private static SynchronousPIDF getAutoAlignHeadingController() {
		SynchronousPIDF controller = new SynchronousPIDF(5.0, 0.0, 0.0);
		controller.setInputRange(-0.5, 0.5);
		controller.setMaxAbsoluteOutput(kMaxAngularRate.in(Units.RotationsPerSecond));
		controller.setContinuous();
		return controller;
	}

	private static SynchronousPIDF getAutoAlignTranslationController() {
		SynchronousPIDF controller = new SynchronousPIDF(3.15, 0.0, 0.0);
		controller.setMaxAbsoluteOutput(kMaxSpeed.in(Units.MetersPerSecond));
		return controller;
	}

    public static final SynchronousPIDF mAutoAlignHeadingController = getAutoAlignHeadingController();
	public static final SynchronousPIDF mAutoAlignTranslationController = getAutoAlignTranslationController();
    
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
