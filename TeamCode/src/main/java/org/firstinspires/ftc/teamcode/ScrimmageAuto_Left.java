package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="ScrimmageAuto_Left", group="Robot")
public class ScrimmageAuto_Left extends ScrimmageAuto {
    public void runOpMode() {
        // Call the parent class method to use its setup
        super.runOpMode();

        //1.6s ≈ 2ft at 0.25speed
        //hang specimen
        driveSeconds(24 * secondsPerInch, 0.25f, dir.RIGHT);
        moveSlideSeconds(3 * slideConst, 0.25f, true);
        driveSeconds(30 * secondsPerInch, 0.25f, dir.BACKWARD);
        moveArm(1);
        moveSlideSeconds(2 * slideConst, 0.25f, false);
        moveClaw(false);

        //move backwards and park
        driveSeconds(3 * secondsPerInch, 0.25f, dir.FORWARD);
        driveSeconds(30 * secondsPerInch, 0.25f, dir.LEFT);
        driveSeconds(24 * secondsPerInch, 0.25f, dir.BACKWARD);
        moveSlideSeconds(1 * slideConst, 0.25f, false);
    }
}
