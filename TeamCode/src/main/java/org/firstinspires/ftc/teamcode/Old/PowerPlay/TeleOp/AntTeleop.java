package org.firstinspires.ftc.teamcode.Old.PowerPlay.TeleOp;

import static java.lang.Math.abs;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Robots.FWDRobot;
@Disabled
@TeleOp(name = "AntTeleop")
public class AntTeleop extends LinearOpMode {
    public RFMotor motorLeftBack;
    public RFMotor motorRightBack;
    public RFMotor motorLeftFront;
    public RFMotor motorRightFront;
    public RFServo clawServo;

    public void runOpMode() {
        FWDRobot robot = new FWDRobot(this, true);

        motorLeftBack = new RFMotor("motorLeftBack", DcMotor.RunMode.RUN_WITHOUT_ENCODER, false);
        motorRightBack = new RFMotor("motorRightBack", DcMotor.RunMode.RUN_WITHOUT_ENCODER, false);
        motorLeftFront = new RFMotor("motorLeftFront", DcMotor.RunMode.RUN_WITHOUT_ENCODER, false);
        motorRightFront = new RFMotor("motorRightFront", DcMotor.RunMode.RUN_WITHOUT_ENCODER, false);
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

        clawServo.setPosition(0.3);

        while (!isStopRequested() && getRuntime() < 90) {

            double left_stick_y = -gamepad1.left_stick_y;
            double left_stick_x = gamepad1.left_stick_x;
            double right_stick_x = -gamepad1.right_stick_x * 0.5;

            move(left_stick_y, left_stick_x, right_stick_x);

            if (gamepad1.right_bumper && getRuntime() - lastpressed > 1) {
                grabrelease();
                lastpressed = getRuntime();
            }

        }
        idle();
    }

    private void move(double left_stick_y, double left_stick_x, double right_stick_x) {
        double max = abs(left_stick_x) + abs(left_stick_y) + abs(right_stick_x);

        motorLeftBack.setPower((left_stick_y - left_stick_x - right_stick_x)/max);
        motorLeftFront.setPower((left_stick_y + left_stick_x - right_stick_x)/max);
        motorRightBack.setPower((left_stick_y + left_stick_x + right_stick_x)/max);
        motorRightFront.setPower((left_stick_y - left_stick_x + right_stick_x)/max);
    }

    private void grabrelease() {
        clawServo.setPosition(0.3 - clawServo.getPosition());
    }
}