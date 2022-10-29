package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "Official Manual Mode", group = "Match")
public class OfficialManualMode extends LinearOpMode {
    private DcMotor _fl, _fr, _rl, _rr;
    private Servo _grip, _platform, _elbow, _shoulder;
    private double _armPosition = 0, _leftTrigger = 0, _rightTrigger = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        _fl = hardwareMap.dcMotor.get("frontLeft");
        _fr = hardwareMap.dcMotor.get("frontRight");
        _rl = hardwareMap.dcMotor.get("rearLeft");
        _rr = hardwareMap.dcMotor.get("rearRight");

        // Reverse the right side motors
        // Reverse left motors if you are using NeveRests
        _fr.setDirection(DcMotorSimple.Direction.REVERSE);
        _rr.setDirection(DcMotorSimple.Direction.REVERSE);

        _platform = hardwareMap.get(Servo.class, "platform");
        _grip = hardwareMap.get(Servo.class, "grip");
        _elbow = hardwareMap.get(Servo.class, "elbow");
        _shoulder = hardwareMap.get(Servo.class, "shoulder");

        waitForStart();
        if (isStopRequested()) return;

        while (opModeIsActive()) {
            controlArm();
            controlWheels();
        }
    }

    private void controlWheels() {
        double y = gamepad1.left_stick_y * 0.5; // Remember, this is reversed!
        double x = -gamepad1.left_stick_x * 1.1 * 0.5; // Counteract imperfect strafing
        double rx = -gamepad1.right_stick_x * 0.5;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;
        double speedmultiplier = 1;
        if (gamepad1.left_trigger > 0) {
            speedmultiplier = 2;
        } else if (gamepad1.right_trigger > 0) {
            speedmultiplier = 0.5;
        } else {
            speedmultiplier = 1;
        }
        _fl.setPower(frontLeftPower * speedmultiplier);
        _rl.setPower(backLeftPower * speedmultiplier);
        _fr.setPower(frontRightPower * speedmultiplier);
        _rr.setPower(backRightPower * speedmultiplier);
        telemetry.addData("FLSpd", frontLeftPower * speedmultiplier);
        telemetry.addData("RRSpd", backRightPower * speedmultiplier);
        telemetry.update();
    }



        private void controlArm(){

        if (gamepad2.left_bumper)
            _platform.setPosition(0.5);
        else if (gamepad2.right_bumper)
            _platform.setPosition(0);
        else if (gamepad2.x)
            _shoulder.setPosition(0.5);
        else if (gamepad2.a)
            _shoulder.setPosition(0);
        else if (gamepad2.b)
            _elbow.setPosition(0.5);
        else if (gamepad2.y)
            _elbow.setPosition(0);
        else if (gamepad2.left_stick_button)
            _grip.setPosition(1);
        else if (gamepad2.right_stick_button)
            _grip.setPosition(0);


        _leftTrigger = gamepad2.left_trigger;
        _rightTrigger = gamepad2.right_trigger;
    }
}
