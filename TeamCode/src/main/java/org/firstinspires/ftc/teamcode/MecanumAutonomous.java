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
        robot.init(hardwareMap, telemetry);
        telemetry.addData("Say", "Hello Driver");

        waitForStart();

        driveBot(10,10, 0.4, 5.0);
        driveBot(-10,-10, 0.4, 5.0);
        move(45, 20, 0.1, 5.0);
        move(225, 20, 0.1, 5.0);
        move(135, 20, 0.1, 5.0);
        move(315, 20, 0.1, 5.0);
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
                telemetry.addData("Left Target:", leftTarget );
                telemetry.addData("Right Target:", rightTarget );
                telemetry.addData("The Motors Are Busy:", robot.motorsBusy());
                telemetry.update();
            }
            //set the motor powers to 0
            robot.setPowers(0.0);
            //stop and reset encoders
            robot.restartEncoders();
        }
    }
    // END driveBot method
    public void move(double degrees, double distance, double power, double timeoutS)
    {
        double degreesToR = Math.toRadians(degrees);
        double x = Math.cos(degreesToR);
        double y = Math.sin(degreesToR);
        double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
        robot.motorFrontLeft.setPower((y + x) / denominator);
        robot.motorBackLeft.setPower((y - x) / denominator);
        robot.motorFrontRight.setPower((y - x) / denominator);
        robot.motorBackRight.setPower((y + x) / denominator);

        int target1 = (int) driveDistance(distance);
        int target2 = (int) driveDistance(distance);


        if(degrees >= 0 && degrees <= 90)
        {
            //both wheels go forward, no change needed
            robot.motorFrontLeft.setTargetPosition(target1);
            robot.motorBackRight.setTargetPosition(target2);
        }
        if(degrees >90 && degrees < 180)
        {
            //both wheels go backwards
            target1 = -target1;
            target2 = -target2;
            robot.motorFrontRight.setTargetPosition(target1);
            robot.motorBackLeft.setTargetPosition(target2);
        }
        if(degrees >= 180 && degrees <= 270)
        {
            //both wheels go backwards
            target1 = -target1;
            target2 = -target2;
            robot.motorFrontLeft.setTargetPosition(target1);
            robot.motorBackRight.setTargetPosition(target2);
        }
        if(degrees > 270 && degrees < 360)
        {
            //both wheels go forward, no change needed
            robot.motorFrontRight.setTargetPosition(target1);
            robot.motorBackLeft.setTargetPosition(target2);
        }

        robot.setRunToPosition();
        runtime.reset();

        robot.motorFrontLeft.setPower((y + x) / denominator);
        robot.motorBackLeft.setPower((y - x) / denominator);
        robot.motorFrontRight.setPower((y - x) / denominator);
        robot.motorBackRight.setPower((y + x) / denominator);

        while (opModeIsActive() &&
                (runtime.seconds() < timeoutS) &&
                (robot.motorsBusy()))
        {
            telemetry.addData("The Motors Are Busy:", robot.motorsBusy());
            telemetry.update();
        }
        //set the motor powers to 0
        robot.setPowers(0.0);
        //stop and reset encoders
        robot.restartEncoders();
    }

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