package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.HydrAuton;

public class HydrAuton_WingParkOnly extends HydrAuton {
    protected boolean secondDrop = false;
    public boolean RunAuton() {
        if (autonState < 100) {
            // These states handle driving to the correct spike
            if (!AutonDriveToSpike(setTrueForRiggingOnRight)) {
                BadState();
                return true;
            }
        }
        else if (autonState < 200) {
            // These 100 level states handle dropping the pixel on the spike
            // this is the same for all autons
            if (!PixelDrop(secondDrop)) {
                BadState();
                return true;
            }
        }
        else if (autonState < 300) {
            if (!secondDrop) {
                // These 200 level states handle driving to the backstage
                if (!AutonDriveToBackdropFromWing(setTrueForRed, true)) {
                    BadState();
                    return true;
                }
            }
            else {
                // the second time we get here we don't drive again, we are now done
                autonState = 500;
                return true;
            }
        }
        else if (autonState < 400) {
            // Now we need to go back to the pixel drop states, so make sure we skip to the end later
            secondDrop = true;
            autonState = 100;
        }
        else if (autonState < 500) {
            // We should not have these states in this auton
            BadState();
            return true;
        }
        else if (autonState == 500) {
            // this auton is complete
            return true;
        }
        else {
            BadState();
            return true;
        }
        return false;
    }
}
