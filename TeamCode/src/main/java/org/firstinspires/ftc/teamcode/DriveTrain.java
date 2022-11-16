package org.firstinspires.ftc.teamcode;

//import the firstinspires package will all the necessary libraries

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class DriveTrain extends LinearOpMode {

//declare our motors

    //private DcMotor LinearSlide = null;
    private DcMotor LeftFrontMotor = null;
    private DcMotor LeftBackMotor = null;
    private DcMotor RightFrontMotor = null;
    private DcMotor RightBackMotor = null;

    @Override
    public void runOpMode() throws InterruptedException {

        //show status

        telemetry.addData("Status", "Initialized");

        telemetry.update();

        //map our motors with the corresponding configuration names on the Control Hub

        //LinearSlide = hardwareMap.get(DcMotor.class, "LinearSlideMotor");
        LeftFrontMotor = hardwareMap.get(DcMotor.class, "LeftFrontMotor"); //0
        LeftBackMotor = hardwareMap.get(DcMotor.class, "LeftBackMotor"); //2
        RightFrontMotor = hardwareMap.get(DcMotor.class, "RightFrontMotor");//1
        RightBackMotor = hardwareMap.get(DcMotor.class, "RightBackMotor");//3

        LeftFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        LeftBackMotor.setDirection(DcMotor.Direction.REVERSE);
        RightFrontMotor.setDirection(DcMotor.Direction.FORWARD);
        RightBackMotor.setDirection(DcMotor.Direction.REVERSE);
       // LinearSlide.setDirection(DcMotor.Direction.REVERSE);

        // get IMU from hardware map, and set parameters to gather data
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

  /*          boolean finger = false;

            if (gamepad1.right_bumper)
                LinearSlide.setPower(0.3);
            else if (gamepad1.left_bumper)
               LinearSlide.setPower(-0.3);
           else
              LinearSlide.setPower(0);

 */

            double y = -gamepad1.left_stick_x; // Remember, this is reversed!
            double x = gamepad1.left_stick_y * 1.1; // Counteract imperfect strafing
            double rx = gamepad1.right_stick_y;

            // Read inverse IMU heading, as the IMU heading is CW positive
            double botHeading = -imu.getAngularOrientation().firstAngle;

            double rotX = x * Math.cos(botHeading) - y * Math.sin(botHeading);
            double rotY = x * Math.sin(botHeading) + y * Math.cos(botHeading);

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rx) / denominator;
            double backLeftPower = (rotY - rotX + rx) / denominator;
            double frontRightPower = (rotY - rotX - rx) / denominator;
            double backRightPower = (rotY + rotX - rx) / denominator;

            LeftFrontMotor.setPower(frontLeftPower);
            LeftBackMotor.setPower(backLeftPower);
            RightFrontMotor.setPower(frontRightPower);
            RightBackMotor.setPower(backRightPower);
        }
    }
}