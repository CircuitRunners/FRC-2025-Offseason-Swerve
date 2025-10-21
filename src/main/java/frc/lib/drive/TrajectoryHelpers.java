package frc.lib.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import frc.lib.util.FieldLayout.Level;
import frc.lib.util.Util;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.superstructure.SuperstructureConstants;

public class TrajectoryHelpers {

    public static Rotation2d angleToScore(Drive drive, Translation2d translation) {
		Translation2d scoringTranslation = translation;
		return scoringTranslation
				.minus(drive.getPose().getTranslation())
				.getAngle();
	}

    public static Rotation2d angleToApproach(Drive drive, Translation2d translation, Rotation2d idealAngle, Angle deadband) {
                Rotation2d currentAngle = angleToScore(drive, translation);
		double currentDegrees = currentAngle.getDegrees();
		double targetDegrees = idealAngle.getDegrees();

		if (idealAngle.getMeasure().isEquivalent(Units.Degrees.of(180))
				&& currentAngle.getMeasure().lte(Units.Degrees.of(0))) currentDegrees += 360.0;

		return Util.epsilonEquals(currentDegrees, targetDegrees, deadband.in(Units.Degrees))
				? Rotation2d.fromDegrees(currentDegrees)
				: idealAngle.plus(Rotation2d.fromDegrees(
						Math.signum(currentDegrees - targetDegrees) * deadband.in(Units.Degrees)));
	}

    public static Pose2d getDriveTargetPose(Drive drive, Pose2d endEffectorPose, Angle idealApproachDeadband, Level level) {
		Rotation2d angle =
				angleToApproach(drive, endEffectorPose.getTranslation(), endEffectorPose.getRotation(), idealApproachDeadband);
		Pose2d transformedTargetPose = transformWantedGamepieceToDrivePose(
				new Pose2d(endEffectorPose.getTranslation(), angle),
				SuperstructureConstants.kElevatorCenterOffset.plus(
						SuperstructureConstants.getGamepieceOffsetFactor(level)));

		return transformedTargetPose;
	}

    public static Pose2d transformWantedGamepieceToDrivePose(Pose2d wantedFinalPose, Distance offsetLength) {
		Rotation2d r = wantedFinalPose.getRotation();
		Distance x = wantedFinalPose.getMeasureX().minus(offsetLength.times(r.getCos()));
		Distance y = wantedFinalPose.getMeasureY().minus(offsetLength.times(r.getSin()));
		return new Pose2d(x, y, r);
	}
}
