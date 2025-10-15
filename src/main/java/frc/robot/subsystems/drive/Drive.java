package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
/**
 * The class represents the drivetrain of the robot. It manages telemetry output, pose tracking, and updating drivetrains tates.
 */
public class Drive extends SubsystemBase {
    /**The most recently read drivetrain state */
    private SwerveDriveState lastReadState;

    /** The swerve drivetrain controller created from tuner constants */
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();


    /** The maximum speed of the drivetrain in meters per second at 12V */
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);

    /** Telemetry object for keeping drivetrain data */
    private final Telemetry telemetry = new Telemetry(MaxSpeed);

    /** A WPILib Field2d object used to display the robot pose in the dashboard */
    private final Field2d elasticPose = new Field2d();

    /**
     * Constructs the Drive subsystem.
     * Initializes drivetrain state tracking and registers telemetry.
     */
    public Drive() {
        lastReadState = drivetrain.getState();
        drivetrain.registerTelemetry(telemetry::telemeterize);
    }

    @Override
    /**
     * Periodic method that updates the drivetrain state and outputs telemetry to dashboard.
     */
    public void periodic() {
        lastReadState = drivetrain.getState();
        outputTelemetry();
    }
    /**
     * Outputs telemetry and pose data to SmartDashboard.
     */
    public void outputTelemetry() {
        telemetry.telemeterize(lastReadState);
        SmartDashboard.putData("Drive", this);
        elasticPose.setRobotPose(getPose());
        SmartDashboard.putData("Elastic Field 2D", elasticPose);
    }

    /**
     * Returns the {@link CommandSwerveDrivetrain} managed by this subsystem.
     * @return The instance of the drivetrain controller.
     */
    public CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }

    /**
     * Returns the current pose of the robot on the field.
     * @return The robot's pose as a {@link Pose2d}.
     */
    public Pose2d getPose() {
        return lastReadState.Pose;
    }

    /**
     * Returns the current drivetrain state.
     * @return the most recent {@link SwerveDriveState}.
     */
    public SwerveDriveState getState() {
        return drivetrain.getState();
    }

    /**
     * Resets the drivetrain's odometry to a specific pose.
     * @param pose The new pose to reset odometry to.
     */
    public void resetPose(Pose2d pose) {
        getDrivetrain().resetPose(pose);
    }
}