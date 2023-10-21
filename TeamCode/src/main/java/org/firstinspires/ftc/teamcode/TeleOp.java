package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


public class TeleOp extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hw = new Hardware();
        hw.init(hardwareMap);

        telemetry.addData("Init >> ", "Ready for start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            //temporary code
            double drive = curveInput(gamepad1.left_stick_y);
            double turn = curveInput(gamepad1.right_stick_x);
            double strafe = curveInput(gamepad1.left_stick_x);

            double maxPower = Math.max(Math.abs(drive) + Math.abs(turn) + Math.abs(strafe), 1);


            //to do: verify this works
            if (turn >= 0) { //forward and backward with turning and strafing || positive strafe/turn = strafe/turn right
                Hardware.frontLeft.setPower((drive - turn + strafe) / maxPower);
                Hardware.backRight.setPower((drive - turn + strafe) / maxPower);
            } else {
                Hardware.frontLeft.setPower((drive - turn - strafe) / maxPower);
                Hardware.backLeft.setPower((drive - turn - strafe) / maxPower);
            }

            //cycle every 50 milliseconds, to prevent memory death --> 20 cycles/s
            sleep(50);
        }

        //end TeleOp
        Hardware.frontLeft.setPower(0);
        Hardware.backLeft.setPower(0);
        Hardware.frontRight.setPower(0);
        Hardware.backRight.setPower(0);
    }

    public double curveInput(double input) {
        /*
            curve function:
            y = (-2.09 / (1+e^4x)) + 1.04
        */

        return ((-2.09 / (1 + Math.pow(Math.E, 4 * input))) + 1.04);
    }
}
