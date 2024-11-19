package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="RightEncoderAuto", group="Robot")
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
}
