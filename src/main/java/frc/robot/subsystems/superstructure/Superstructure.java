package frc.robot.subsystems.superstructure;

import edu.wpi.first.math.util.Units;

public class Superstructure {
    private boolean isPathFollowing = false;
    private boolean superstructureDone = false;
    private boolean driveReady = false;
    
    public void setDriveReady(boolean valToSet) {
		driveReady = valToSet;
	}

    public void setPathFollowing(boolean isFollowing) {
		  isPathFollowing = isFollowing;
	}

    public void setSuperstructureDone(boolean valToSet) {
		superstructureDone = valToSet;
	}


    
}
