package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOp")
public class TeleOp extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {
        Hardware hw = new Hardware();
        hw.init(hardwareMap);

        telemetry.addData("TeleOp Init: ", "Ready for start");
        telemetry.update();

        waitForStart();

        telemetry.addData("TeleOp: ", "starting...");
        telemetry.update();

        while (opModeIsActive()) {
            //temporary code
            telemetry.addData("Teleop: ", "working");
            telemetry.update();
            /*double drive = curveInput(gamepad1.left_stick_y);
            double turn = curveInput(gamepad1.right_stick_x);
            double strafe = curveInput(gamepad1.left_stick_x);*/

            double drive = gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;
            double strafe = gamepad1.left_stick_x;

            double maxPower = Math.max(Math.abs(drive) + Math.abs(turn) + Math.abs(strafe), 1);


            //to do: verify this works
            if (turn > 0) { //forward and backward with turning and strafing || positive strafe/turn = strafe/turn right
                //hw.frontLeft.setPower((drive - turn + strafe) / maxPower);
                telemetry.addData("Teleop:", "test");
                telemetry.update();
                hw.backRight.setPower(0.25);
                //hw.backRight.setPower((drive - turn + strafe) / maxPower);
            } else if(turn < 0){
                hw.frontLeft.setPower((drive - turn - strafe) / maxPower);
                hw.backLeft.setPower((drive - turn - strafe) / maxPower);
            } else {
                hw.frontLeft.setPower(drive + strafe);
                hw.frontRight.setPower(drive - strafe);
                hw.backLeft.setPower(drive + strafe);
                hw.backRight.setPower(drive - strafe);
            }

            //cycle every 50 milliseconds, to prevent memory death --> 20 cycles/s
            sleep(50);
        }

        //end TeleOp
        hw.frontLeft.setPower(0);
        hw.backLeft.setPower(0);
        hw.frontRight.setPower(0);
        hw.backRight.setPower(0);
    }

    public double curveInput(double input) {
        /*
            curve function:
            y = (-2.09 / (1+e^4x)) + 1.04
        */

        return ((-2.09 / (1 + Math.pow(Math.E, 4 * input))) + 1.04);
    }
}
