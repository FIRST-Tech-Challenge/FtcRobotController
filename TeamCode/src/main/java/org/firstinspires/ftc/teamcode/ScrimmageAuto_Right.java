package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="ScrimmageAuto_Right", group="Robot")
public class ScrimmageAuto_Left extends ScrimmageAuto {
    public void runOpMode() {
        // Call the parent class method to use its setup
        super.runOpMode();

        //1.6s ≈ 2ft at 0.25speed
        //hang specimen
        driveSeconds(24 * secondsPerInch, 0.25f, dir.LEFT);
        moveSlideSeconds(3 * slideConst, 0.25f, true);
        driveSeconds(30 * secondsPerInch, 0.25f, dir.BACKWARD);
        moveSlideSeconds(2 * slideConst, 0.25f, false);
        //moveClaw(false);

        //move backwards and park
        driveSeconds(3 * secondsPerInch, 0.25f, dir.FORWARD);
        driveSeconds(30 * secondsPerInch, 0.25f, dir.RIGHT);
        driveSeconds(24 * secondsPerInch, 0.25f, dir.BACKWARD);
        moveSlideSeconds(1 * slideConst, 0.25f, false);
    }


/*  OLD CODE/////////////////////////////////
public class ScrimmageAuto_Right extends ScrimmageAuto {
    public void runOpMode() {
        // Call the parent class method to use its setup
        super.runOpMode();
        driveInches(24, 1, dir.LEFT, 5);
        moveSlideSeconds(20, 1, true);
        driveInches(7, 1, dir.FORWARD, 5);
        moveSlideSeconds(2, 1, false);
        moveClaw(false);
        moveSlideSeconds(2, 1, true);
        driveInches(3, 1, dir.BACKWARD, 5);
        driveInches(30, 1, dir.RIGHT, 5);
        driveInches(24, 1, dir.FORWARD, 5);
        moveSlideSeconds(12, 1, false);
    }
*/
}
