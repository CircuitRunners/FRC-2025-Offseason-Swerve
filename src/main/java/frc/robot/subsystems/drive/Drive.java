package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

@Logged
public class Drive extends SubsystemBase {
    private SwerveDriveState lastReadState;
    private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private final Telemetry telemetry = new Telemetry(MaxSpeed);

    private final Field2d elasticPose = new Field2d();

    public Drive() {
        lastReadState = drivetrain.getState();
        drivetrain.registerTelemetry(telemetry::telemeterize);
    }

    @Override
    public void periodic() {
        lastReadState = drivetrain.getState();
        outputTelemetry();
    }

    public void outputTelemetry() {
        telemetry.telemeterize(lastReadState);
        SmartDashboard.putData("Drive", this);
        elasticPose.setRobotPose(getPose());
        SmartDashboard.putData("Elastic Field 2D", elasticPose);
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