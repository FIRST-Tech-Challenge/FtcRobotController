package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "lakshyaCode")
public class TeleOp_Mode extends OpMode {
    Lift theBetterLift;
    Chassis theBetterChassis;
    double x;
    double y;
    double rx;

    @Override
    public void init() {
        theBetterChassis = new Chassis(hardwareMap);
        theBetterLift = new Lift(hardwareMap);

        theBetterLift.lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    @Override
    public void loop() {
        y = applyResponseCurve(gamepad1.left_stick_y);
        x = -applyResponseCurve(gamepad1.left_stick_x);
        rx = -applyResponseCurve(gamepad1.right_stick_x);

        telemetry.addData("Lift Encoder", theBetterLift.encoderValue());
        telemetry.addData("Left stick y", gamepad1.left_stick_y);
        telemetry.addData("Left stick x", gamepad1.left_stick_x);
        telemetry.addData("Right stick x", gamepad1.right_stick_x);
        telemetry.addData("Adjusted x", x);
        telemetry.addData("Adjusted y", y);
        telemetry.addData("Adjusted rx", rx);

        if (gamepad1.y) {
            theBetterChassis.resetYaw();
        }

        theBetterChassis.drive(x, y, rx);
        theBetterLift.drive(-gamepad2.left_stick_y);

        telemetry.update();
    }

    public double applyResponseCurve(double input) {
        double exponent = 2;
        return Math.signum(input) * Math.pow(Math.abs(input), exponent);
    }

}