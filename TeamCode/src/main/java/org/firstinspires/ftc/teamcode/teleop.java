package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// LinearOpMode seems to be what most of this works off of
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
// this too

@TeleOp(name = "Teleop", group = "Teleop")
// this is the thing that we run
public class teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        double speed = .5;
        // this sets the speed

        Training Train = new Training();
        // im pretty sure this defines "train"

        Train.init(this);
        // train

        waitForStart();

        while (opModeIsActive()) {
            // (pretty much while this is running)
            if (gamepad1.left_stick_y < -.4) {
                // run the forward function from Training program
                Train.forward();
            } else if (gamepad1.left_stick_y > .4) {
                // backwards
                Train.backwards();
            } else if (gamepad1.right_stick_x < -.4) {
                // left
                Train.left();
            } else if (gamepad1.right_stick_x > .4) {
                // right
            Train.right();

                // run the stop function from training
            } else {
                Train.stop();
            }
        }
    }
}
// train