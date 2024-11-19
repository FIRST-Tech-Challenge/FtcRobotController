package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="LeftEncoderAuto", group="Robot")
public class ScrimmageAuto_Left extends ScrimmageAuto {
    private final double driveConst = 1;
    private final double slideConst = 1;
    public void runOpMode() {
        // Call the parent class method to use its setup
        super.runOpMode();
        /*
        /*driveInches(24, 0.25f, dir.FORWARD, 50);
        driveInches(24, 0.25f, dir.BACKWARD, 50);
        driveInches(24, 0.25f, dir.RIGHT, 50);
        driveInches(24, 0.25f, dir.LEFT, 50);

        moveSlide(20, 1, true, 5); * /
        //driveInches(24, 0.25f, dir.RIGHT, 5);
        driveInches(1, 0.25f, dir.FORWARD, 5);

        //moveSlide(2, 1, false, 1);
     // moveClaw(false);
      //  moveSlide(2, 1, true, 1);
        driveInches(1, 0.25f, dir.BACKWARD, 5);
        //driveInches(30, 0.25f, dir.LEFT, 5);
        //driveInches(24, 0.25f, dir.FORWARD, 5);
        moveSlide(12, 1, false, 1);*/

        //1.6s ≈ 2ft at 0.25speed
        driveSeconds(1.6 * driveConst, 0.25f, dir.LEFT);
        moveSlideSeconds(1 * slideConst, 0.25f, true);
        driveSeconds(0.467 * driveConst, 0.25f, dir.FORWARD);
        moveSlideSeconds(1 * slideConst, 0.25f, false);
        //moveClaw(false);
        moveSlideSeconds(2 * slideConst, 0.25f, true);
        driveSeconds(0.2 * driveConst, 0.25f, dir.BACKWARD);
        driveSeconds(2 * driveConst, 0.25f, dir.RIGHT);
        driveSeconds(1.6 * driveConst, 0.25f, dir.FORWARD);
        moveSlideSeconds(1 * slideConst, 0.25f, false);
    }
}
