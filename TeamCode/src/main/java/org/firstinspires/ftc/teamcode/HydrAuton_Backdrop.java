package org.firstinspires.ftc.teamcode;

public class HydrAuton_Backdrop extends HydrAuton {
    protected boolean setTrueForRed = false;
    protected boolean setTrueForRiggingOnRight = false;
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
            if (!AutonDriveToBackdropFromBackstage(setTrueForRed)) {
                BadState();
                return true;
            }
        }
        else if (autonState < 400) {
            // These 300 level states handle scoring backwards at the backdrop
            // this is the same for two autons
            if (!ScoreBack()) {
                BadState();
                autonState = 400;
            }
        }
        else if (autonState < 500) {
            // These 400 level states handle returning the arm home
            // this is the same for all autons
            ArmToHome();
        }
        else if (autonState < 600) {
            switch (autonState) {
                case 500:
                    if (!Drive.Busy()) {
                        int flip = 1;
                        if (setTrueForRed) {
                            flip = -1;
                        }
                        Drive.Start(0, -20 * flip, 0);
                        autonState += 501;
                    }
                    break;
                case 501:
                    if (!Drive.Busy()) {
                        autonState = 600;
                    }
                    break;
                default:
                    BadState();
                    return false;
            }
        }
        else if (autonState == 600) {
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
