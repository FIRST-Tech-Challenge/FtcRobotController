package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Hardware2022;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheels;

@TeleOp(name="GeneralDriver2022", group="TeleOps")

public class GeneralDriver extends BaseTele {

    Hardware2022 hdw;

    MecanumWheels robotWheel;

    @Override
    public void runOpMode() throws InterruptedException {
        hdw = new Hardware2022(hardwareMap); //init hardware
        hdw.createHardware();
        robotWheel = new MecanumWheels();

        hdw.wheelFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.Encoders.setPosition(1.0);

        double powerDrivePercentage = 0.5;

        telemetry.addData("[>]", "All set?");
        telemetry.update();

        //hdw.servoRingCounter.setPosition(0.0);


        waitForStart();

        //This is the main loop of operation.
        while (opModeIsActive()) {
            if (gamepad1.start) {
            }
            if (gamepad2.dpad_up) {
                hdw.Encoders.setPosition(1.0);
                sleep(100);
            }
            if (gamepad2.dpad_down) {
                hdw.Encoders.setPosition(0.35);
                sleep(100);
            }
            if (gamepad2.y) {

            }
            if (gamepad2.a) {

            }

            if (gamepad2.x) {

            }

            //Wheel takes input of gampad 1  ,  turbo is the power factor. Range 0-1 , 1 is 100%
            robotWheel.joystick(gamepad1, 1);

            float powerTwoRight = gamepad1.right_trigger;
            float powerTwoLeft = gamepad1.left_trigger;

            /* This is important for the driving to work ⬇⬇ */
            hdw.wheelFrontRight.setPower(robotWheel.wheelFrontRightPower * powerDrivePercentage);
            hdw.wheelFrontLeft.setPower(robotWheel.wheelFrontLeftPower * powerDrivePercentage);
            hdw.wheelBackRight.setPower(robotWheel.wheelBackRightPower * powerDrivePercentage);
            hdw.wheelBackLeft.setPower(robotWheel.wheelBackLeftPower * powerDrivePercentage);

            telemetry.addData("[>]", "All set?");
            telemetry.update();

        }

    }

}
