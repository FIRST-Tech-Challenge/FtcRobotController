package org.firstinspires.ftc.teamcode.Old.PowerPlay.TeleOp;

import static java.lang.Math.abs;
import static java.lang.Math.cos;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFMotor;
import org.firstinspires.ftc.teamcode.Components.RFModules.Devices.RFServo;
import org.firstinspires.ftc.teamcode.Old.FreightFrenzy.Robots.TWDRobot;
@Disabled
@TeleOp(name = "AnteaterTeleop")
public class AnteaterTeleop extends LinearOpMode {

    public RFMotor motorLeft;
    public RFMotor motorRight;
    public RFMotor armMotor;
    public RFServo clawServo;

    private final double ARM_GRAVITY_CONSTANT = 0.362;
    private final double TICKS_PER_RADIAN = 84;

    public void runOpMode() {
        TWDRobot robot = new TWDRobot(this, true);

        motorLeft = new RFMotor("motorLeft", DcMotor.RunMode.RUN_WITHOUT_ENCODER, false);
        motorRight = new RFMotor("motorRight", DcMotor.RunMode.RUN_WITHOUT_ENCODER, false);
        armMotor = new RFMotor("armMotor", DcMotor.RunMode.RUN_WITHOUT_ENCODER, false);
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

        armMotor.setPosition(0);

        while (!isStopRequested() && getRuntime() < 90) {

            double left_stick_y = gamepad1.left_stick_y;
            double right_stick_x = gamepad1.right_stick_x*1.5;

            move(left_stick_y, right_stick_x);

            if (gamepad1.right_bumper && getRuntime() - lastpressed > 1) {
                grabrelease();
                lastpressed = getRuntime();
            }

            raise(-gamepad1.right_trigger, -gamepad1.left_trigger);

            telemetry.addData("arm power", gamepad1.right_trigger);
            telemetry.addData("arm pos", armMotor.getCurrentPosition());
            telemetry.update();

        }
        idle();
    }

    private void move(double left_stick_y, double right_stick_x) {
        double max = abs(left_stick_y) + abs(right_stick_x);

        motorLeft.setPower((left_stick_y - right_stick_x)/3);
        motorRight.setPower((-left_stick_y - right_stick_x)/3);
        //-0.3, 0.874, -45
    }

    private void raise(double right_trigger, double left_trigger) {
        double power = -ARM_GRAVITY_CONSTANT * cos((armMotor.getCurrentPosition()+18)/TICKS_PER_RADIAN) +
                (right_trigger-left_trigger)*0.25;
        armMotor.setPower(power);
        telemetry.addData("power", power);
    }

    private void grabrelease() {
        clawServo.setPosition(0.4 - clawServo.getPosition());
    }
}