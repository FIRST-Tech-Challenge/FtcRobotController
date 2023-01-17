package org.firstinspires.ftc.teamcode;

import android.util.Log;

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

        double powerDrivePercentage = 0.5 ;

        telemetry.addData("[>]", "All set?");
        telemetry.update();

        waitForStart();
        telemetry.clearAll();


        //This is the main loop of operation.
        while (opModeIsActive()) {
            //hdw.checkAndGrabCone();

            if (gamepad1.dpad_right && gamepad1.x) {
                sleep (400);
                if ( hdw.iseMode()) {
                    hdw.seteMode(false);
                    telemetry.addLine("eMode off (Limits on slide)");
                    telemetry.update();

                } else {
                    hdw.seteMode(true);
                    telemetry.addLine("eMode on (No limits on slide)");
                    telemetry.update();

                }
            }
            if (gamepad1.left_bumper) {
                hdw.goToHeight(Hardware2022.SlideHeight.Mid);
            }
            if (gamepad1.right_bumper) {
                hdw.goToHeight(Hardware2022.SlideHeight.High);
                Log.d("9010", "After go to vSLide HIGH");
            }
            if (gamepad1.dpad_down) {
                hdw.goToHeight(Hardware2022.SlideHeight.Low);
            }


            //hdw.freeLowerVerticalSlide(gamepad1.left_trigger);
            hdw.freeMoveVerticalSlide(gamepad1.right_trigger - gamepad1.left_trigger);

            //Wheel takes input of gampad 1  ,  turbo is the power factor. Range 0-1 , 1 is 100%
            robotWheel.joystick(gamepad1, 1);

            /* Set the calcuated velocity to wheels according to the gampad input */
            double frontLeftVelocity = robotWheel.wheelFrontLeftPower * powerDrivePercentage * Hardware2022.ANGULAR_RATE;
            double backLeftVelocity = robotWheel.wheelBackLeftPower * powerDrivePercentage * Hardware2022.ANGULAR_RATE;
            double frontRightVelocity = robotWheel.wheelFrontRightPower * powerDrivePercentage * Hardware2022.ANGULAR_RATE;
            double backRightVelocity = robotWheel.wheelBackRightPower * powerDrivePercentage * Hardware2022.ANGULAR_RATE;

            hdw.wheelFrontLeft.setVelocity(frontLeftVelocity);
            hdw.wheelBackLeft.setVelocity(backLeftVelocity);
            hdw.wheelFrontRight.setVelocity(frontRightVelocity);
            hdw.wheelBackRight.setVelocity(backRightVelocity);

            /*
            telemetry.addData("Front left set ",frontLeftVelocity );
            telemetry.addData("back left set",backLeftVelocity );
            telemetry.addData("Front right set",frontRightVelocity);
            telemetry.addData("back right set",backRightVelocity );

            telemetry.addData("fl: " , hdw.wheelFrontLeft.getVelocity() + " mode:" + hdw.wheelFrontLeft.getMode()
            + " port: " +  hdw.wheelFrontLeft.getPortNumber());
            telemetry.addData("bl: " , hdw.wheelBackLeft.getVelocity() + " mode:" + hdw.wheelBackLeft.getMode()
                    + " port: " +  hdw.wheelBackLeft.getPortNumber());
            telemetry.addData("fr: " , hdw.wheelFrontRight.getVelocity() + " mode:" + hdw.wheelFrontRight.getMode()
                    + " port: " +  hdw.wheelFrontRight.getPortNumber());
            telemetry.addData("br: " , hdw.wheelBackRight.getVelocity() + " mode:" + hdw.wheelBackRight.getMode()
                    + " port: " +  hdw.wheelBackRight.getPortNumber());

            telemetry.update();
            */

        }

    }

}
