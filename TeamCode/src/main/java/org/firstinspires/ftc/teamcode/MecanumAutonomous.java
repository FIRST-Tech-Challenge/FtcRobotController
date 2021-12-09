package org.firstinspires.ftc.teamcode;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.external.libs.PIDController;
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

    //Declare the PIDController and other variables for it.
    PIDController pidRotate;
    Orientation lastAngles = new Orientation();
    double correction;
    float Yaw_Angle = 0;

    //True if Y is pressed
    boolean startPositionDuck = false;
    //True if X is pressed
    boolean redAlliance = false;
    //True if A is pressed
    boolean endPositionDuck = false;
    private boolean aPressed;
    private boolean yPressed;
    private boolean xPressed;

    @Override
    public void runOpMode() {
        //Import the robot's hardware map
        robot.init(hardwareMap, telemetry);

        // Initialize IMU
        IMU_Parameters = new BNO055IMU.Parameters();
        IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        robot.imu.initialize(IMU_Parameters);
        IMU_Calibrated();

        /* Set PID proportional value to start reducing power at about 50 degrees of rotation.
        P by itself may stall before turn completed so we add a bit of I (integral) which
        causes the PID controller to gently increase power if the turn is not completed.
        */
        pidRotate = new PIDController(.003, .00003, 0);

        /*
        Selection code for starting position and alliance settings.
        Booleans descriptions are True / False
        startPositionDuck is for selecting Duck Side / Warehouse Side.
        redAlliance is for Red Alliance / Blue Alliance
        endPositionDuck is for selecting Duck Side / Warehouse Side.
        */
        while(! isStarted()) {
            if (gamepad1.y & !yPressed) {
                startPositionDuck = !startPositionDuck;
                telemetry.addData("Y Pressed", "pressed");
            }
            //Set pressed variable so when the loop runs next frame an input only registers if it is new.
            yPressed = gamepad1.y;
            if (gamepad1.x & !xPressed) {
                redAlliance = !redAlliance;
            }
            xPressed = gamepad1.x;
            if (gamepad1.a & !aPressed) {
                endPositionDuck = !endPositionDuck;
                telemetry.addData("A Pressed", "pressed");
            }
            aPressed = gamepad1.a;

            //Print input telemetry.
            telemetry.addData("Start Position", "Y = Start Position \n X = Alliance \n A = End Position");
            telemetry.addData("Settings", "\n%s, %s, %s",
                    startPositionDuck ? "startDuck": "startWarehouse",
                    redAlliance ? "Red" : "Blue",
                    endPositionDuck ?  "endDuck" : "endWarehouse");
            telemetry.update();
        }

        //Wait for the input's start.
        waitForStart();

        //Run code while the opMode is active.
        if(opModeIsActive()) {
            driveStraight(50,0.8,5.0);
            drive(0,25, 0.8, 5.0);
            drive(0,25,-0.8,5.0);
            rotate(90, 0.8);
            rotate(-90, 0.8);
        }
    }

    /*
    Drive forward/backward. Travel distance in CM.
    @param distanceInCM Distance to be driven (In centimeters)
    @param power Motor power (From 0.0 to 1.0)
    @param timeoutS Motor movement timeout (Adjust accordingly, or just put 5)
    */
    public void driveStraight(double distanceInCM, double power, double timeoutS) {
        telemetry.addData("status","encoder reset");
        telemetry.update();
        robot.restartEncoders();
        if(opModeIsActive()) {
            telemetry.addData("status","getEncoderClicks");
            telemetry.update();
            //Set motor target, reset runtime for timeout, then set motor powers.
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
            //Loop while motors are moving towards target.
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
            //Set the motor powers to 0.
            robot.setPowers(0.0);
            //Stop and reset encoders.
            robot.restartEncoders();
        }
    }

    /*
    Drive in any direction.
    @param degrees Direction to be driven (In degrees) (From 0 to 360)
    @param distance Distance to be driven (In centimeters)
    @param power Motor power (From 0.0 to 1.0)
    @param timeout Motor movement timeout (In Seconds) (Adjust accordingly, or just put 5)
    */
    public void drive(double degrees, double distance, double power, double timeout) {
        //Math to convert input(degrees) into x and y.
        double degreesToR = Math.toRadians(degrees);
        double x = Math.cos(degreesToR);
        double y = Math.sin(degreesToR);
        double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
        //Convert the input distance into something useable by encoders.
        int target = (int) driveDistance(distance);
        int direction = 1;
        double slowdown = 1;
        //Set motor targets based on direction.
        if(degrees >= 0 && degrees <= 90) {
            //Both wheels go backwards.
            target = -target;
            robot.motorFrontLeft.setTargetPosition(target);
            robot.motorBackRight.setTargetPosition(target);
            direction = 1;
        }
        if(degrees >90 && degrees < 180) {
            //Both wheels go forward, no change needed.
            robot.motorFrontRight.setTargetPosition(target);
            robot.motorBackLeft.setTargetPosition(target);
            direction = 2;
        }
        if(degrees >= 180 && degrees <= 270) {
            //Both wheels go forward, no change needed.
            robot.motorFrontLeft.setTargetPosition(target);
            robot.motorBackRight.setTargetPosition(target);
            direction = 3;
        }
        if(degrees > 270 && degrees < 360) {
            //Both wheels go backwards.
            target = -target;
            robot.motorFrontRight.setTargetPosition(target);
            robot.motorBackLeft.setTargetPosition(target);
            direction = 4;
        }
        //Set the motors' target and begin runtime for timeout.
        robot.setRunToPosition();
        runtime.reset();
        //Set initial motor powers.
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

    /*
    Calculate drive distance to be used in driving methods by converting centimeters into encoder clicks.
    @param distance Distance to be driven (In centimeters)
    */
    public double driveDistance(double distance) {
        double drive  = (robot.REV_ENCODER_CLICKS/ robot.REV_WHEEL_CIRC);
        return (int)Math.floor(drive * distance);
    }

    //Reset IMU angle calculations
    private void resetAngle() {
        lastAngles = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        globalAngle = 0;
    }

    //Get the IMU's global angle.
    private double getAngle() {
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

    /*
    Turn the robot (Counterclockwise).
    @param degrees Amount of degrees to be turned (0 to 360)
    */
    private void rotate(int degrees, double power) {
        //Reset IMU's angle tracking.
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
        pidRotate.reset();
        pidRotate.setSetpoint(degrees);
        pidRotate.setInputRange(0, degrees);
        pidRotate.setOutputRange(0, power);
        pidRotate.setTolerance(1);
        pidRotate.enable();
        //getAngle() returns a positive number when rotating left and a negative number  when rotating right.
        //First if statement is for a right turn with negative power.
        if (degrees < 0) {
            //Right turns have to start from 0.
            while (opModeIsActive() && getAngle() == 0) {
                robot.motorBackLeft.setPower(power);
                robot.motorFrontLeft.setPower(power);
                robot.motorBackRight.setPower(-power);
                robot.motorFrontRight.setPower(-power);
                sleep(100);
            }
            do {
                power = pidRotate.performPID(getAngle()); //power will be - on right turn.
                robot.motorBackLeft.setPower(power);
                robot.motorFrontLeft.setPower(power);
                robot.motorBackRight.setPower(-power);
                robot.motorFrontRight.setPower(-power);
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 turn rotation", rotation);
            } while (opModeIsActive() && !pidRotate.onTarget());
        }
        //Else statement is for left turn with positive power.
        else
            do {
                power = pidRotate.performPID(getAngle());
                robot.motorBackLeft.setPower(power);
                robot.motorFrontLeft.setPower(power);
                robot.motorBackRight.setPower(-power);
                robot.motorFrontRight.setPower(-power);
                telemetry.addData("1 imu heading", lastAngles.firstAngle);
                telemetry.addData("2 global heading", globalAngle);
                telemetry.addData("3 correction", correction);
                telemetry.addData("4 turn rotation", rotation);
                telemetry.update();
            } while (opModeIsActive()&& !pidRotate.onTarget());
        //Turn off the motors.
        robot.setPowers(0);
        //Get the robot's angle.
        rotation = getAngle();
        //Sleep so rotation can stop.
        sleep(500);
        //Reset IMU's angle tracking.
        resetAngle();
    }

    //Returns telemetry for IMU Calibration.
    public void IMU_Calibrated() {
        telemetry.addData("IMU Calibration Status", robot.imu.getCalibrationStatus());
        telemetry.addData("Gyro Calibrated", robot.imu.isGyroCalibrated() ? "True" : "False");
        telemetry.addData("System Status", robot.imu.getSystemStatus().toString());
        telemetry.update();
    }
}