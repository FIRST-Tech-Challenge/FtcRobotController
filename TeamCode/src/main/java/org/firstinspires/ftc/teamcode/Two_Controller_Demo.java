package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
// Hi from github
@TeleOp(name = "2 Controller")
public class Two_Controller_Demo extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        // Uses [Z] Motor Config
        DcMotor motorFrontLeft = hardwareMap.dcMotor.get("frontLeft");
        DcMotor motorFrontRight = hardwareMap.dcMotor.get("frontRight");
        DcMotor motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        DcMotor motorBackRight = hardwareMap.dcMotor.get("backRight");

        DcMotor spineL = hardwareMap.dcMotor.get("spinLeft");
        DcMotor spineR = hardwareMap.dcMotor.get("spinRight");

        // Reverse the right side motors
        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        int speedlimit = 2;

//        boolean dpad_stat = false;

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

//            if ((gamepad2.dpad_up) && (!dpad_stat)){
//                speedlimit = speedlimit + 1;
//                dpad_stat = true;
//            } else if ((gamepad2.dpad_down) && (!dpad_stat) && (speedlimit != 1)) {
//                speedlimit = speedlimit - 1;
//                dpad_stat = true;
//            }
//
//            if (!gamepad2.dpad_up || !gamepad2.dpad_down){
//                dpad_stat = false;
//            }

            if (gamepad1.a == true){
                spineL.setPower(0.5217391304347826);
                spineL.setPower(1);
            } else {
                spineL.setPower(0);
                spineL.setPower(0);
            }

            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator / speedlimit;
            double backLeftPower = (y - x + rx) / denominator / speedlimit;
            double frontRightPower = (y - x - rx) / denominator / speedlimit;
            double backRightPower = (y + x - rx) / denominator / speedlimit;

            // Some Silly Telemetry
            telemetry.addData("Gamepad X", x);
            telemetry.addData("Gamepad Y", y);
            telemetry.addData("Speed Limit Factor", speedlimit);

            // Updates my silly little telemetry
            telemetry.update();

            motorFrontLeft.setPower(frontLeftPower);
            motorBackLeft.setPower(backLeftPower);
            motorFrontRight.setPower(frontRightPower);
            motorBackRight.setPower(backRightPower);

        }
    }
}