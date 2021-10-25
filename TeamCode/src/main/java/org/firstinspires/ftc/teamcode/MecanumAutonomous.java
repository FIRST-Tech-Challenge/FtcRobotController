package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="MecanumAutonomous", group="FreightFrenzy")
public class MecanumAutonomous extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    /*
    get the robot's hardware map outside of runOpMode
    this is so functions outside of runOpMode can also access it
    */
    FrenzyHardwareMap robot = new FrenzyHardwareMap();

    @Override
    public void runOpMode() {

        //import the hardware map
        robot.init(hardwareMap);
        telemetry.addData("Say", "Hello Driver");

        waitForStart();

        //double y = -gamepad1.left_stick_y; // Remember, this is reversed!
        //double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
        //double rx = gamepad1.right_stick_x;

        // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]

        /*double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        robot.motorFrontLeft.setPower(frontLeftPower);
        robot.motorBackLeft.setPower(backLeftPower);
        robot.motorFrontRight.setPower(frontRightPower);
        robot.motorBackRight.setPower(backRightPower);
        */

        driveBot(10,10, 0.4, 5.0);
        driveBot(-10,-10, 0.4, 5.0);
    }
    //END OF RUN OPMODE


    /**
     * Drive Method
     * Left & Right travel distance in CM +/-, power to both wheels, timeout in seconds
     */
    public void driveBot(double distanceInCMleft, double distanceInCMright, double power, double timeoutS)
    {
        telemetry.addData("status","encoder reset");
        telemetry.update();
        robot.restartEncoders();

        int rightTarget;
        int leftTarget;

        if(opModeIsActive())
        {
            telemetry.addData("status","getEncoderClicks");
            telemetry.update();

            rightTarget = (int) driveDistance(distanceInCMright);
            leftTarget = (int) driveDistance(distanceInCMleft);

            robot.setTargets(leftTarget, rightTarget);
            robot.setRunToPosition();

            runtime.reset();

            robot.setPowers(power);

            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.motorsBusy()))
            {
                telemetry.addData("Path1", "leftTarget, rightTarget" );
                telemetry.update();
            }
            //set the motor powers to 0
            robot.setPowers(0.0);
            //stop and reset encoders
            robot.restartEncoders();
        }
    }
    // END driveBot method


    /*
         Calculate DriveDistance for DriveBot method
    */
    public double driveDistance(double distance)
    {
        double drive  = (robot.REV_ENCODER_CLICKS/ robot.REV_WHEEL_CIRC);
        return (int)Math.floor(drive * distance);
    }

    /*
        IMU Calibration Method
        Checks IMU calibration and returns telementry
        If IMU is NOT calibrated, run the calibration opMode
    */
    public boolean IMU_Calibrated() {
        telemetry.addData("IMU Calibration Status", robot.imu.getCalibrationStatus());
        telemetry.addData("Gyro Calibrated", robot.imu.isGyroCalibrated() ? "True" : "False");
        telemetry.addData("System Status", robot.imu.getSystemStatus().toString());
        return robot.imu.isGyroCalibrated();
    }
}