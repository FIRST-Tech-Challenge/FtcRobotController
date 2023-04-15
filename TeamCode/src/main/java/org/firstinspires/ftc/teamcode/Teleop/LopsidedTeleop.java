package org.firstinspires.ftc.teamcode.Teleop;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Old.Robots.TWDRobot;

@TeleOp(name = "LopsidedTeleop")
public class LopsidedTeleop extends LinearOpMode {

    public RFMotor motorLeft;
    public RFMotor motorRight;
    public RFServo clawServo;

    public void runOpMode() {
        TWDRobot robot = new TWDRobot(this, true);

        motorLeft = new RFMotor("motorLeft", DcMotor.RunMode.RUN_WITHOUT_ENCODER, false);
        motorRight = new RFMotor("motorRight", DcMotor.RunMode.RUN_WITHOUT_ENCODER, false);
        clawServo = new RFServo("clawServo", 1.0);

        telemetry.addData("Status", "Before new Robot");
        telemetry.update();

        telemetry.addData("Status", "Done with new Robot");
        telemetry.update();

        telemetry.addData("Status", "Ready to go");
        telemetry.update();

        while (!opModeIsActive() && !isStopRequested()) {
            telemetry.addData("status", "waiting for start command...");
            telemetry.update();
        }

        double lastpressed = 0;

        while (!isStopRequested() && getRuntime() < 90) {

            double left_stick_y = gamepad1.left_stick_y;
            double right_stick_x = gamepad1.right_stick_x*1.5;

            move(left_stick_y, right_stick_x);

            if (gamepad1.right_bumper && getRuntime() - lastpressed > 1) {
                grabrelease();
                lastpressed = getRuntime();
            }
        }
        idle();
    }

    private void move(double left_stick_y, double right_stick_x) {
        double max = abs(left_stick_y) + abs(right_stick_x);

        motorLeft.setPower((left_stick_y - right_stick_x)/3);
        motorRight.setPower((-left_stick_y - right_stick_x)/3);
    }

    private void grabrelease() {
        clawServo.setPosition(0.4 - clawServo.getPosition());
    }
}