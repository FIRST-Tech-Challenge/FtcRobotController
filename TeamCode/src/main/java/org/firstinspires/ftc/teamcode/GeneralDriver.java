package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.hardware.Hardware2022;
import org.firstinspires.ftc.teamcode.hardware.MecanumWheels;

@TeleOp(name="GeneralDriver2022", group="TeleOps")
public class GeneralDriver extends BaseTele {

    private boolean debug = true;
    Hardware2022 hdw;

    MecanumWheels robotWheel;

    @Override
    public void runOpMode() throws InterruptedException {
        hdw = new Hardware2022(hardwareMap, telemetry); //init hardware
        hdw.createHardware();
        robotWheel = new MecanumWheels();

        hdw.wheelFrontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelFrontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelBackRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hdw.wheelBackLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double powerDrivePercentage = 1;

        telemetry.addData("[>]", "All set?");
        telemetry.update();

        //hdw.servoRingCounter.setPosition(0.0);

        telemetry.addData("[>]", "All set?");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();


        //This is the main loop of operation.
        while (opModeIsActive()) {
            //hdw.checkAndGrabCone();

            if (gamepad1.dpad_left) {

            }
            if (gamepad1.dpad_right) {
                hdw.goToHeight(Hardware2022.SlideHeight.Mid);
            }
            if (gamepad2.dpad_up) {
                hdw.goToHeight(Hardware2022.SlideHeight.High);
            }
            if (gamepad2.dpad_down) {
                hdw.goToHeight(Hardware2022.SlideHeight.Low);
            }
            if (gamepad1.y) {
                hdw.releaseCone();

            }
            if (gamepad1.b) {
                hdw.lowerVerticalSlide();
            }

            if (gamepad1.x) {
                hdw.manualgrab();
            }

            //hdw.freeLowerVerticalSlide(gamepad1.left_trigger);
            hdw.freeMoveVerticalSlide(gamepad1.right_trigger - gamepad1.left_trigger);

            //Wheel takes input of gampad 1  ,  turbo is the power factor. Range 0-1 , 1 is 100%
            robotWheel.joystick(gamepad1, 1);

            /* Set the calcuated power to wheels according to the gampad input */
            hdw.wheelFrontRight.setPower(robotWheel.wheelFrontRightPower * powerDrivePercentage);
            hdw.wheelFrontLeft.setPower(robotWheel.wheelFrontLeftPower * powerDrivePercentage);
            hdw.wheelBackRight.setPower(robotWheel.wheelBackRightPower * powerDrivePercentage);
            hdw.wheelBackLeft.setPower(robotWheel.wheelBackLeftPower * powerDrivePercentage);


        }

    }

}
