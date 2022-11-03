package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
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

        double powerDrivePercentage = 0.5;

        telemetry.addData("[>]", "All set?");
        telemetry.update();

        //hdw.servoRingCounter.setPosition(0.0);

        telemetry.addData("[>]", "All set?");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();


        //This is the main loop of operation.
        while (opModeIsActive()) {
            hdw.checkAndGrabCone();

            if (gamepad1.start) {
            }
            if (gamepad2.dpad_up) {

                sleep(100);
            }
            if (gamepad2.dpad_down) {
                sleep(100);
            }
            if (gamepad1.y) {
               hdw.moveYAxis(2160, -0.5);

            }
            if (gamepad1.b) {
                if ( debug) {
                    telemetry.addLine().addData("[>]  ", "Relase cone, open claw.");
                    telemetry.update();
                }
                hdw.releaeCone();
            }

            if (gamepad1.x) {
                hdw.moveXAxis( 10.0, -0.5 );
                sleep(6000);
            }

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
