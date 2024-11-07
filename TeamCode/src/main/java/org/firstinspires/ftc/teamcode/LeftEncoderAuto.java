package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="LeftEncoderAuto", group="Robot")
@Disabled
public class LeftEncoderAuto extends MainEncoderAuto {
    public void runOpMode() {
        // Call the parent class method to use its setup
        super.runOpMode();
        driveInches(24, 1, dir.LEFT, 5);
        moveSlide(20, 1, true, 5);
        driveInches(7, 1, dir.FORWARD, 5);
        moveSlide(2, 1, false, 1);
        moveClaw(false);
        moveSlide(2, 1, true, 1);
        driveInches(3, 1, dir.BACKWARD, 5);
        driveInches(30, 1, dir.RIGHT, 5);
        driveInches(24, 1, dir.FORWARD, 5);
        moveSlide(12, 1, false, 1);
    }
}