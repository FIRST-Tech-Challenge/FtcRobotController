package org.firstinsires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * Made by Pearl Kamalu on June 3, following the tutorial on
 * game manual 0 for field centric controls.
 * 
 * https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
 *  
 * Single controler Operation Mode
 * 
 */

 @TeleOp

public class FieldCentricMecanumPreliminaryOpMode extends LinearOpMode {

    // Primatives and References
private Gryroscope imu;

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    @Override
    public void runOpMode () {
        // Set up IMU
        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
            RevHubOrientationOnRobot.LogoFacingDirection.UP,
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        ));
        imu.initalize(parameters);

        // Set Up Motors, but we need to set these up and make sure these work.
        frontLeftMotor = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRightMotor = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeftMotor = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRightMotor = hardwareMap.get(DcMotor.class, "backRightMotor");

        // Reverse the motors and see if they work
        frontRightMotor.setDirecton(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirecton(DcMotorSimple.Direction.REVERSE);

        // Telemetry
        telemetry.addData("Status", "Initalized");
        telemetry.update();

        waitForStart(); 
        while (opModeIsActive()) {
            // Variables
            // Left stick is used for controling planar motion
            // Right stick is used for turn, but only using x axis to get variable speed

            // Strafing is slower than forwards b/c of lost friction, reqiring more power and thus scaling
            // Adjust this value for driver's preverence
            double strafeScale = 1.2; 

            double forwardPower = -gamepad1.left_stick_y; // Y is reversed
            double strafePower = gamepad1.left_stick_x * strafeScale; // Strafe = sideways power.

            double turnPower = gamepad1.right_stick_x; 

            if (gamepad1.options) {
                imu.resetYaw();
                // Reset yaw button because of drift. 
                // Is options button to make it hard to hit.
            }

            // Calculate for headings
            double robotHeading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
            
            double rotationalX = forwardPower * Math.cos(-robotHeading) - y * Math.sin(-robotHeading);
            double rotationalY = forwardPower * Math.sin(-robotHeading) + y * Math.cos(-robotHeading);


            // SDK clips power at 1, so we need to scale everything else down
            // Divde everything by largest number, so largest number will be one
            double denominator = Math.max(Math.abs(rotationalX) + Math.abs(rotationalY) + Math.abs(turnPower), 1);

            // See gm0's page for mecanum wheels for why.
                // Make `forwardPower` negative if this is the fact.
                // for troubleshooting, comment out everything after forward power and see if they go in correct direction
            double frontLeftMotorPower = (rotationalY + rotationalX + turnPower) / denominator; 
            double frontRightMotorPower = (rotationalY - rotaiotnalX - turnPower) / denominator;
            double backLeftMotorPower = (rotationalY - rotationalX - turnPower) / denominator;     
            double backRightMotorPower =  (rotationalY + rotationalX - turnPower) / denominator;
            
            // See gm0's page for mecanum wheels for why.
            // Make `forwardPower` negative if this is the fact.
            // for troubleshooting, comment out everything after forward power and see if they go in correct direction
            frontLeftMotor.setPower(frontLeftMotorPower);
            frontRightMotor.setPower(frontRightMotorPower);
            backLeftMotor.setPower(backLeftMotorPower);
            backRightMotor.setPower(backRightMotorPower);

        }
    }
}