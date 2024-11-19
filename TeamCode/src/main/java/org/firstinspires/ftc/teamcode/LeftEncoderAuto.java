package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="LeftEncoderAuto", group="Robot")
public class LeftEncoderAuto extends MainEncoderAuto {
    public void runOpMode() {
        // Call the parent class method to use its setup
        super.runOpMode();
        /*driveInches(24, 0.25f, dir.FORWARD, 50);
        driveInches(24, 0.25f, dir.BACKWARD, 50);
        driveInches(24, 0.25f, dir.RIGHT, 50);
        driveInches(24, 0.25f, dir.LEFT, 50);

        moveSlide(20, 1, true, 5);*/
        //driveInches(24, 0.25f, dir.RIGHT, 5);
        driveInches(1, 0.25f, dir.FORWARD, 5);

        //moveSlide(2, 1, false, 1);
     // moveClaw(false);
      //  moveSlide(2, 1, true, 1);
        driveInches(1, 0.25f, dir.BACKWARD, 5);
        //driveInches(30, 0.25f, dir.LEFT, 5);
        //driveInches(24, 0.25f, dir.FORWARD, 5);
     //   moveSlide(12, 1, false, 1);
    }
}
