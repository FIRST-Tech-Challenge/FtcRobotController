package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware.Hardware2022;

@TeleOp(name="HDWTestOp", group="TeleOps")
public class HWTestTele  extends LinearOpMode {
    Hardware2022 hdw;

    @Override
    public void runOpMode() throws InterruptedException {
        hdw = new Hardware2022(hardwareMap, telemetry); //init hardware
        hdw.createHardware();

        telemetry.addData("[>]", "All set?");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();


        //This is the main loop of operation.
        while (opModeIsActive()) {
            //hdw.checkAndGrabCone();

            if (gamepad1.dpad_left) {
                hdw.moveXAxis(-2.0, -1);
            }
            if (gamepad1.dpad_right) {
                hdw.moveXAxis(12.0, 1);
            }
            if (gamepad1.dpad_up) {
                telemetry.addLine().addData("[moving y >]  ", " Y ");
                telemetry.update();

                hdw.moveYAxis(12.0, 1);
            }
            if (gamepad1.dpad_down) {
                hdw.moveYAxis (-2.0, -1);
            }

            if (gamepad1.a) {
                hdw.turn(90);
            }

            if ( gamepad1.b) {
                hdw.turn(-90);
            }

        }
    }

}
