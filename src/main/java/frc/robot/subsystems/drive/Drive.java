package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import java.security.Timestamp;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
    private SwerveDriveState lastReadState;
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final Telemetry logger = new Telemetry(MaxSpeed);

    public Drive() {
        lastReadState = drivetrain.getState();
        drivetrain.registerTelemetry(logger::telemeterize);
    }

    @Override
    public void periodic() {
        lastReadState = drivetrain.getState();
    }

    public CommandSwerveDrivetrain getDrivetrain() {
        return drivetrain;
    }

    public Pose2d getPose() {
        return lastReadState.Pose;
    }

    public SwerveDriveState getState() {
        return drivetrain.getState();
    }

    public void resetPose(Pose2d pose) {
        getDrivetrain().resetPose(pose);
    }
}