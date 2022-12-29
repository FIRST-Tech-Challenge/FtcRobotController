package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="SingleMotorEncoder", group="Linear OpMode")
//@Disabled
public class SingleMotorEncoder extends LinearOpMode
{
    DcMotor motor;

    @Override
    public void runOpMode() throws InterruptedException {
        motor = hardwareMap.dcMotor.get("leftFront");

        motor.setDirection(DcMotor.Direction.FORWARD);

        // reset encoder counts kept by motors.
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // set motors to run forward for 5000 encoder counts.
        motor.setTargetPosition(100);

        // set motors to run to target encoder position and stop with brakes on.
        motor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("Mode", "waiting");
        telemetry.update();

        // wait for start button.

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        // set both motors to 25% power. Movement will start. Sign of power is
        // ignored as sign of target encoder position controls direction when
        // running to position.

        motor.setPower(0.25);

        // wait while opmode is active and left motor is busy running to position.

        while (opModeIsActive() && motor.isBusy())
        {
            telemetry.addData("Encoder Position", motor.getCurrentPosition() + "  busy = " + motor.isBusy());

            telemetry.update();
            idle();
        }

        // set motor power to zero to turn off motors. The motors stop on their own but
        // power is still applied so we turn off the power.

        motor.setPower(0.0);
    }


}