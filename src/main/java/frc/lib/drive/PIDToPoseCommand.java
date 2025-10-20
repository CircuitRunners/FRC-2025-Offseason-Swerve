package frc.lib.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.DelayedBoolean;
import frc.lib.util.SynchronousPIDF;
import frc.lib.util.Util;

public class PIDToPoseCommand extends Command {
    Pose2d finalpose;
    Rotation2d targetRotation;
    Distance epsilonDist;
    Angle epsilonAngle;
    Util.Pose2dTimeInterpolable interpolable;
    DelayedBoolean atTarget;
    boolean isAuto;
    SynchronousPIDF translationController;
    SynchronousPIDF headingController;
}
