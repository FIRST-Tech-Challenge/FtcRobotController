package org.firstinspires.ftc.teamcode;

public class HydrAuton_Wing extends HydrAuton {
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
            if (!PixelDrop()) {
                BadState();
                return true;
            }
        }
        else if (autonState < 300) {
            // These 200 level states handle driving to the backdrop
            if (!AutonDriveToBackdropFromWing(setTrueForRed)) {
                BadState();
                return true;
            }
        }
        else if (autonState < 400) {
            // These 300 level states handle scoring forwards at the backdrop
            // this is the same for two autons
            if (!ScoreFront()) {
                BadState();
                return true;
            }
        }
        else if (autonState < 500) {
            // These 400 level states handle returning the arm home
            // this is the same for all autons
            ArmToHome();
        }
        else if (autonState == 500) {
            // this auton is complete
            return true;
        }
        else {
            BadState();
            return true;
        }
        if (autonState >= 200) {
            // now that we are driving towards the backdrop, start looking for the AprilTag
            // this just prints to telemetry for now
            ObjDet.FindAprilTag(ObjLoc);
        }
        return false;
    }
}
