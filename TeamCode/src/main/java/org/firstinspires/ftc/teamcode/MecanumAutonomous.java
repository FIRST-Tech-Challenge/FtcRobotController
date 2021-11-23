package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
@Autonomous(name="MecanumAutonomous", group="FreightFrenzy")
public class MecanumAutonomous extends LinearOpMode {
    //Add an ElapsedTime for function runtime calculations.
    private ElapsedTime runtime = new ElapsedTime();
    //Import the robot's hardware map.
    FrenzyHardwareMap robot = new FrenzyHardwareMap();
    // Declare IMU
    BNO055IMU.Parameters IMU_Parameters;
    double globalAngle;
    double rotation;
    //PIDController pidRotate;
    Orientation lastAngles = new Orientation();
    double correction;
    float Yaw_Angle = 0;
    boolean startPositionDuck = false;
    //yellow button (y) sets to true
    boolean redAlliance = false;
    //blue button (x) sets to true
    boolean endPositionDuck = false;
    //green button (a) sets to true
    private boolean aPressed;
    private boolean yPressed;
    private boolean xPressed;
    @Override
    public void runOpMode() {
        while(! isStarted()) {
            if (gamepad1.y & !yPressed) {
                startPositionDuck = !startPositionDuck;
                telemetry.addData("ypressed", "pressed");
            }
            yPressed = gamepad1.y;
            if (gamepad1.x & !xPressed) {
                redAlliance = !redAlliance;
            }
            xPressed = gamepad1.x;
            if (gamepad1.a & !aPressed) {
                endPositionDuck = !endPositionDuck;
                telemetry.addData("apressed", "pressed");
            }
            aPressed = gamepad1.a;
            telemetry.addData("Start Position", "y=startPositionDuck / x=redAlliance / a=endPositionDuck");
            telemetry.addData("Settings", "\n%s, %s, %s",
                    startPositionDuck ? "startDuck" : "startWarehouse",
                    redAlliance ? "Blue" : "Red",
                    endPositionDuck ? "endDuck" : "endWarehouse");
            telemetry.update();
            sleep(500);
            }
        // Set PID proportional value to start reducing power at about 50 degrees of rotation.
        // P by itself may stall before turn completed so we add a bit of I (integral) which
        // causes the PID controller to gently increase power if the turn is not completed.
        //pidRotate = new PIDController(.003, .00003, 0);
        telemetry.addData("y=startPositionDuck", "");
        telemetry.addData("Settings", "\n%s, %s, %s",
                startPositionDuck ? "startDuck" : "startWarehouse",
                redAlliance ? "Blue" : "Red",
                endPositionDuck ? "endDuck" : "endWarehouse");
        telemetry.update();
        sleep(10000);
        //Run during program's init (anything before waitForStart()).
        //Import the hardware map
        robot.init(hardwareMap, telemetry);
        telemetry.addData("Say", "Hello Driver");
        // Initialize IMU
        IMU_Parameters = new BNO055IMU.Parameters();
        IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        robot.imu.initialize(IMU_Parameters);
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
        turnRight();
        turnLeft();
    }
    //Drive forward/backward. Travel distance in CM.
    public void forward(double distanceInCM, double power, double timeoutS) {
        telemetry.addData("status","encoder reset");
        telemetry.update();
        robot.restartEncoders();
        if(opModeIsActive()) {
            telemetry.addData("status","getEncoderClicks");
            telemetry.update();
            //Set motor target, reset runtime for timeout, then set motor powers
            int target = (int) driveDistance(distanceInCM);
            boolean forward = true;
            if(distanceInCM < 0) {
                forward = false;
            }
            double slowdown = 1;
            runtime.reset();
            robot.setTargets(target, target);
            robot.setRunToPosition();
            robot.setPowers(power);
            //Loop while motors are moving towards target
            while (opModeIsActive() && (runtime.seconds() < timeoutS) && (robot.motorsBusy())) {
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
    private void resetAngle()
    {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }
    private double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.
        Orientation angles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;
        globalAngle += deltaAngle;
        lastAngles = angles;
        return globalAngle;
    }
    private void rotate(int degrees, double power)
    {
        // restart imu angle tracking.
        resetAngle();
        // if degrees > 359 we cap at 359 with same sign as original degrees.
        if (Math.abs(degrees) > 359) degrees = (int) Math.copySign(359, degrees);
        // start pid controller. PID controller will monitor the turn angle with respect to the
        // target angle and reduce power as we approach the target angle. This is to prevent the
        // robots momentum from overshooting the turn after we turn off the power. The PID controller
        // reports onTarget() = true when the difference between turn angle and target angle is within
        // 1% of target (tolerance) which is about 1 degree. This helps prevent overshoot. Overshoot is
        // dependant on the motor and gearing configuration, starting power, weight of the robot and the
        // on target tolerance. If the controller overshoots, it will reverse the sign of the output
        // turning the robot back toward the setpoint value.
        /*pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();
         */
        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).
        // rotate until turn is completed.
        if (degrees < 0)
        {
            // On right turn we have to get off zero first.
            while (opModeIsActive() && getAngle() == 0)
            {
                robot.motorBackLeft.setPower(power);
                robot.motorFrontLeft.setPower(power);
                robot.motorBackRight.setPower(-power);
                robot.motorFrontRight.setPower(-power);
                sleep(100);
            }

            do
            {
                //power = pidRotate.performPID(getAngle()); // power will be - on right turn.
                robot.motorBackLeft.setPower(-power);
                robot.motorFrontLeft.setPower(-power);
                robot.motorBackRight.setPower(power);
                robot.motorFrontRight.setPower(power);
            } while (opModeIsActive());// && !pidRotate.onTarget());
        }
        else    // left turn.
            do
            {
                //power = pidRotate.performPID(getAngle()); // power will be + on left turn.
                robot.motorBackLeft.setPower(-power);
                robot.motorFrontLeft.setPower(-power);
                robot.motorBackRight.setPower(power);
                robot.motorFrontRight.setPower(power);
            } while (opModeIsActive());// && !pidRotate.onTarget());
        // turn the motors off.
        robot.setPowers(0);
        rotation = getAngle();
        // wait for rotation to stop.
        sleep(500);
        // reset angle tracking on new heading.
        resetAngle();
    }
    //Turn using the IMU.
    public void turn() {
        robot.imu.initialize(IMU_Parameters);
        Yaw_Angle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        //MOVE RIGHT
        robot.motorFrontLeft.setPower(0.2);
        robot.motorBackLeft.setPower(0.2);
        robot.motorFrontRight.setPower(-0.2);
        robot.motorBackRight.setPower(-0.2);
        // Continue until robot yaws right by 90 degrees or stop is pressed on Driver Station.
        sleep(1000);
        //turns to the right; 90, 180, negative, -90, 0
        while ( !(Yaw_Angle >= 88 || isStopRequested()) ) {
            // Update Yaw-Angle variable with current yaw.
            Yaw_Angle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            // Report yaw orientation to Driver Station.
            telemetry.addData("Yaw value", Yaw_Angle);
            telemetry.update();
        }
        // We're done. Turn off motors
        robot.setPowers(0);
        // Pause so final telemetry is displayed.
        sleep(1000);
        robot.imu.initialize(IMU_Parameters);
    }
    //Turn left using the IMU.
    public void turnLeft() {
        Yaw_Angle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        //MOVE LEFT
        robot.motorFrontLeft.setPower(-0.2);
        robot.motorBackLeft.setPower(-0.2);
        robot.motorFrontRight.setPower(0.2);
        robot.motorBackRight.setPower(0.2);
        // Continue until robot yaws right by 90 degrees or stop is pressed on Driver Station.
        sleep(1000);
        //turns to the right; 90, 180, negative, -90, 0
        while ( !(Yaw_Angle <= -88 || isStopRequested()) ) {
            // Update Yaw-Angle variable with current yaw.
            Yaw_Angle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            // Report yaw orientation to Driver Station.
            telemetry.addData("Yaw value", Yaw_Angle);
            telemetry.update();
        }
        // We're done. Turn off motors
        robot.setPowers(0);
        // Pause so final telemetry is displayed.
        sleep(1000);
        robot.imu.initialize(IMU_Parameters);
    }
    //Turn right using the IMU.
    public void turnRight() {
        robot.imu.initialize(IMU_Parameters);
        Yaw_Angle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
        //MOVE RIGHT
        robot.motorFrontLeft.setPower(0.2);
        robot.motorBackLeft.setPower(0.2);
        robot.motorFrontRight.setPower(-0.2);
        robot.motorBackRight.setPower(-0.2);
        // Continue until robot yaws right by 90 degrees or stop is pressed on Driver Station.
        sleep(1000);
        //turns to the right; 90, 180, negative, -90, 0
        while ( !(Yaw_Angle >= 88 || isStopRequested()) ) {
            // Update Yaw-Angle variable with current yaw.
            Yaw_Angle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES).thirdAngle;
            // Report yaw orientation to Driver Station.
            telemetry.addData("Yaw value", Yaw_Angle);
            telemetry.update();
        }
        // We're done. Turn off motors
        robot.setPowers(0);
        // Pause so final telemetry is displayed.
        sleep(1000);
        robot.imu.initialize(IMU_Parameters);
    }
    //Rturns telemetry for IMU Calibration.
    public void IMU_Calibrated() {
        telemetry.addData("IMU Calibration Status", robot.imu.getCalibrationStatus());
        telemetry.addData("Gyro Calibrated", robot.imu.isGyroCalibrated() ? "True" : "False");
        telemetry.addData("System Status", robot.imu.getSystemStatus().toString());
        telemetry.update();
    }
}