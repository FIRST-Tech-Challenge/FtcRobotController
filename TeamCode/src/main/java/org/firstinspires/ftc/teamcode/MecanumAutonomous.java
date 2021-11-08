package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name="MecanumAutonomous", group="FreightFrenzy")
public class MecanumAutonomous extends LinearOpMode {
    //Add an ElapsedTime for function runtime calculations.
    private ElapsedTime runtime = new ElapsedTime();
    //Import the robot's hardware map.
    FrenzyHardwareMap robot = new FrenzyHardwareMap();
    @Override
    public void runOpMode() {
        //Run during program's init (anything before waitForStart()).
        //Import the hardware map
        robot.init(hardwareMap, telemetry);
        telemetry.addData("Say", "Hello Driver");
        IMU_Calibrated();
        //Begin after program's start
        waitForStart();
        //Start of actual code for movement
        forward(20, 0.8, 5.0);
        forward(-20, 0.8, 5.0);
        move(45, 40, 0.8, 5.0);
        move(225, 40, 0.8, 5.0);
        move(135, 40, 0.8, 5.0);
        move(315, 40, 0.8, 5.0);
    }
    //Drive forward/backward. Travel distance in CM.
    public void forward(double distanceInCM, double power, double timeoutS) {
        telemetry.addData("status","encoder reset");
        telemetry.update();
        robot.restartEncoders();
        if(opModeIsActive())
        {
            telemetry.addData("status","getEncoderClicks");
            telemetry.update();
            //Set motor target, reset runtime for timeout, then set motor powers
            int target = (int) driveDistance(distanceInCM);
            boolean forward = true;
            if(distanceInCM < 0)
            {
                forward = false;
            }
            double slowdown = 1;
            runtime.reset();
            robot.setTargets(target, target);
            robot.setRunToPosition();
            robot.setPowers(power);
            //Loop while motors are moving towards target
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.motorsBusy()))
            {
                if(forward && robot.motorFrontLeft.getCurrentPosition() > target*0.75 && robot.motorFrontRight.getCurrentPosition() > target*0.75)
                slowdown = 0.25;
                if(!forward && robot.motorFrontLeft.getCurrentPosition() < target*0.75 && robot.motorFrontRight.getCurrentPosition() < target*0.75)
                slowdown = 0.25;
                robot.setPowers(power*slowdown);
                telemetry.addData("Target", target);
                telemetry.addData("The Motors Are Busy:", robot.motorsBusy());
                telemetry.update();
            }
            //Set the motor powers to 0
            robot.setPowers(0.0);
            //Stop and reset encoders
            robot.restartEncoders();
        }
    }
    //Method which allows the robot to move in any direction (based on degrees). Distance is in CM.
    public void move(double degrees, double distance, double power, double timeout) {
        //Math to convert input(degrees) into x and y
        double degreesToR = Math.toRadians(degrees);
        double x = Math.cos(degreesToR);
        double y = Math.sin(degreesToR);
        double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
        //Convert the input distance into something useable by encoders
        int target = (int) driveDistance(distance);
        int direction = 1;
        double slowdown = 1;
        //Set motor targets based on direction
        if(degrees >= 0 && degrees <= 90) {
            //Both wheels go forward, no change needed
            robot.motorFrontLeft.setTargetPosition(target);
            robot.motorBackRight.setTargetPosition(target);
            direction = 1;
        }
        if(degrees >90 && degrees < 180) {
            //Both wheels go backwards
            target = -target;
            robot.motorFrontRight.setTargetPosition(target);
            robot.motorBackLeft.setTargetPosition(target);
            direction = 2;
        }
        if(degrees >= 180 && degrees <= 270) {
            //Both wheels go backwards
            target = -target;
            robot.motorFrontLeft.setTargetPosition(target);
            robot.motorBackRight.setTargetPosition(target);
            direction = 3;
        }
        if(degrees > 270 && degrees < 360) {
            //Both wheels go forward, no change needed
            robot.motorFrontRight.setTargetPosition(target);
            robot.motorBackLeft.setTargetPosition(target);
            direction = 4;
        }
        //Set the motors' target and begin runtime for timeout
        robot.setRunToPosition();
        runtime.reset();
        //Set initial motor powers
        robot.motorFrontLeft.setPower(((y + x) / denominator)*power);
        robot.motorBackLeft.setPower(((y - x) / denominator)*power);
        robot.motorFrontRight.setPower(((y - x) / denominator)*power);
        robot.motorBackRight.setPower(((y + x) / denominator)*power);
        //Loop while the motors move to their target position
        while (opModeIsActive() && (runtime.seconds() < timeout) && (robot.motorsBusy())) {
            //Add slowdown to robot if distance traveled is over 75% of the way there (this changes based on direction)
            if(direction == 1 && robot.motorFrontLeft.getCurrentPosition() > distance*0.75)
                slowdown = 0.25;
            if(direction == 2 && robot.motorFrontRight.getCurrentPosition() < distance*0.75)
                slowdown = 0.25;
            if(direction == 3 && robot.motorFrontLeft.getCurrentPosition() < distance*0.75)
                slowdown = 0.25;
            if(direction == 4 && robot.motorFrontRight.getCurrentPosition() > distance*0.75)
                slowdown = 0.25;
            //Calculate the power for the motors (takes direction and multiplies it by slowdown and power)
            double newSpeed = slowdown*power;
            double input1 = ((y + x) / denominator) * newSpeed;
            double input2 = ((y - x) / denominator) * newSpeed;
            //Set the motors to their correct powers
            robot.motorFrontLeft.setPower(input1);
            robot.motorBackLeft.setPower(input2);
            robot.motorFrontRight.setPower(input2);
            robot.motorBackRight.setPower(input1);
            //Display telemetry for motor business
            telemetry.addData("The Motors Are Busy:", robot.motorsBusy());
            telemetry.update();
        }
        //Stop the motors, then reset the encoder counts
        robot.setPowers(0.0);
        robot.restartEncoders();
    }
    //Calculate DriveDistance for DriveBot method.
    public double driveDistance(double distance) {
        double drive  = (robot.REV_ENCODER_CLICKS/ robot.REV_WHEEL_CIRC);
        return (int)Math.floor(drive * distance);
    }
    //Rturns telemetry for IMU Calibration.
    public void IMU_Calibrated() {
        telemetry.addData("IMU Calibration Status", robot.imu.getCalibrationStatus());
        telemetry.addData("Gyro Calibrated", robot.imu.isGyroCalibrated() ? "True" : "False");
        telemetry.addData("System Status", robot.imu.getSystemStatus().toString());
        telemetry.update();
    }
}