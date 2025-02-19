package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Limit Switch Experiment", group="A")
public class testingLimitSwitch extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor slideMotor = null;
    TouchSensor touchSensor;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        slideMotor = hardwareMap.get(DcMotor.class, "motor");
        slideMotor.setDirection(DcMotor.Direction.FORWARD);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        touchSensor = hardwareMap.get(TouchSensor.class, "touch");

        boolean stop = false;
        int basePosition = 0;
        double slidePower = 0;
        int ticks = 0;

        waitForStart();

        while (opModeIsActive()) {
            ticks++;
            int slidePosition = slideMotor.getCurrentPosition();
            boolean override = gamepad1.a;

            if (touchSensor.isPressed() && stop == false) {
                stop = true;
                basePosition = slidePosition;
            }
            if (stop == true && slidePosition < basePosition-100) {
                stop = false;
            }

            double pow = 1;
            if (gamepad1.b) {
                pow = 5;
            } else {
                pow = 1;
            }

            if (stop == false || override == true) {slidePower = gamepad1.left_stick_y/pow;}
            else                   {slidePower = Math.min(gamepad1.left_stick_y/pow, 0);}

            telemetry.addData("Motor Power", "(%1.2f)", slidePower);
            telemetry.addData("Base Position", basePosition);
            telemetry.addData("Slide Position", slidePosition);
            if (touchSensor.isPressed()) {
                telemetry.addLine("Touch Sensor Is Pressed");
            } else {
                telemetry.addLine("Touch Sensor Is Not Pressed");
            }
            telemetry.update();

            slideMotor.setPower(slidePower);

        }
    }
}
