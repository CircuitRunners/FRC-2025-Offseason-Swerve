package frc.lib.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.DelayedBoolean;
import frc.lib.util.SynchronousPIDF;
import frc.lib.util.Util;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.superstructure.SuperstructureConstants;

public class PIDToPoseCommand extends Command {
    Drive drive;
    Pose2d finalPose;
    Rotation2d targetRotation;
    Distance epsilonDist;
    Angle epsilonAngle;
    Util.Pose2dTimeInterpolable interpolable;
    DelayedBoolean atTarget;
    boolean isAuto;
    SynchronousPIDF translationController;
    SynchronousPIDF headingController;

    public PIDToPoseCommand(
            Drive drive,
            Pose2d finalPose,
            Distance epsilonDist,
            Angle epsilonAngle,
            Time delayTime,
            Rotation2d targetRotation,
            SynchronousPIDF translationController,
            SynchronousPIDF headingController) {
        addRequirements(drive);

        this.drive = drive;
        this.finalPose = finalPose;
		this.epsilonDist = epsilonDist;
		this.epsilonAngle = epsilonAngle;
		this.targetRotation = targetRotation.plus(Rotation2d.k180deg);
		this.translationController = translationController;
		this.headingController = headingController;

        atTarget = new DelayedBoolean(Timer.getFPGATimestamp(), delayTime.in(Units.Seconds));
    }

    public PIDToPoseCommand(
            Drive drive,
			Pose2d finalPose,
			Distance epsilonDist,
			Angle epsilonAngle,
			Time delayTime,
			SynchronousPIDF translationController,
			SynchronousPIDF headingController) {
        this(
                drive,
                finalPose,
                epsilonDist,
                epsilonAngle,
                delayTime,
                finalPose.getRotation(),
                translationController,
                headingController);
    }

    public PIDToPoseCommand(
            Drive drive,
            Pose2d finalPose,
            Time delayTime,
            Rotation2d targetRotation,
            SynchronousPIDF translationController,
            SynchronousPIDF headingController) {
        this(
                drive,
                finalPose,
                Units.Centimeters.of(4),
                Units.Degrees.of(.8),
                delayTime,
                targetRotation,
                translationController,
                headingController);
    }

    public PIDToPoseCommand(
            Drive drive,
			Pose2d finalPose,
			Time delayTime,
			SynchronousPIDF translationController,
			SynchronousPIDF headingController) {
		this(drive, finalPose, delayTime, finalPose.getRotation(), translationController, headingController);
	}

    /**
	 * Boolean param was added to avoid confusion with the constructor that would transform a scoring pose from a branch.
	 * For example, this is used for our auto align to L1.
	 */
	// public PIDToPoseCommand(Pose2d rawEndPose, Level level, boolean useRaw) {
	// 	this(
	// 			// rawEndPose.transformBy(new Transform2d(new Translation2d(), Rotation2d.k180deg)),
	// 			// SuperstructureConstants.getAutoAlignScoringDistanceEpsilon(level),
	// 			// SuperstructureConstants.getAutoAlignScoringAngleEpsilon(level),
	// 			// SuperstructureConstants.getAutoAlignScoringDelay(level),
	// 			// rawEndPose.getRotation(),
	// 			// DriveConstants.mAutoAlignTranslationController,
	// 			// DriveConstants.mAutoAlignHeadingController);
	// }


}
