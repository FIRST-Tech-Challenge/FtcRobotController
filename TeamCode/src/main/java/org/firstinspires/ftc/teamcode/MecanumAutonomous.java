package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.external.libs.PIDController;
import org.firstinspires.ftc.teamcode.hardwaremaps.FrenzyHardwareMap;

@Disabled
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

    private double intakePowerOut = 0.4;
    @Override
    public void runOpMode() {
        //Import the robot's hardware map
        robot.init(hardwareMap, telemetry);
        // Initialize IMU
        IMU_Parameters = new BNO055IMU.Parameters();
        IMU_Parameters.mode = BNO055IMU.SensorMode.IMU;
        robot.imu.initialize(IMU_Parameters);
        IMU_Calibrated();
        /*
        Set PID proportional value to start reducing power at about 50 degrees of rotation.
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
            //Set pressed variable so when the loop runs next frame an input only registers if it is new.
            xPressed = gamepad1.x;
            if (gamepad1.a & !aPressed) {
                endPositionDuck = !endPositionDuck;
                telemetry.addData("A Pressed", "pressed");
            }
            //Set pressed variable so when the loop runs next frame an input only registers if it is new.
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

            // Test 0 degrees forward/back
            drive(0, 30, 0.2, 4);
            drive(180, 30, 0.2, 4);

            // 90 degree square
            drive(-90, 40, 0.2, 4); // right strafe
            drive(0, 40, 0.2, 4); // forward
            drive(90, 40, 0.2, 4); // left strafe
            drive(180, 40, 0.2, 4); // back

            // 45 degree diagonals
            drive(45, 10, 0.2, 4); // diagonal right forward
            drive(-45, 10, 0.2, 4); // diagonal left forward
            drive(-135, 10, 0.2, 4); // diagonal right back
            drive(135, 10, 0.2, 4); // diagonal left back

            // Check weird angles
            drive(21, 10, 0.2, 4);
            drive(98, 10, 0.2, 4);
            drive(-22, 10, 0.2, 4);
            drive(-126, 10, 0.2, 4);

            /*
           driveStraight(25,0.8,5.0);
           moveArm(600,0.5);
           sleep(1000);
           moveArm(0,0.5);
           sleep(1000);
           drive(0,25, 0.8, 1.0);
           sleep(2000);
           drive(45,25, 0.8, 1.0);
           sleep(2000);
           drive(90,25, 0.8, 1.0);
           */


           /*
           drive(45,25, 0.8, 5.0);
           rotate(45, 0.8);
           robot.motorIntake.setPower(intakePowerOut);
           robot.motorCarousel.setPower(0);
           moveArm(0,0.5);
           rotate(-45, 0.8);
           */

        }
    }
    //Sets the arm to move to position with power
    public void moveArm(int armSetPos, double power){
        robot.motorArm.setTargetPosition(armSetPos);
        robot.motorArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorArm.setPower(0.5);
    }
    /*
    Drive forward/backward. Travel distance in CM.
    @param distanceInCM Distance to be driven (In centimeters)
    @param power Motor power (From 0.0 to 1.0)
    @param timeoutS Motor movement timeout (Adjust accordingly, or just put 5)
    */
    public void driveStraight(double distanceInCM, double power, double timeoutS) {
        distanceInCM = -distanceInCM;
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
    @param degrees Direction to be driven (In degrees) (From 0 to 359)
    @param distance Distance to be driven (In centimeters)
    @param power Motor power (From 0.0 to 1.0)
    @param timeout Motor movement timeout (In Seconds) (Adjust accordingly, or just put 5)
    */
    public void drive(double degrees, double distance, double power, double timeout) {
        double degreesToR = Math.toRadians(degrees + 90); // flip sine/cosine -90 degrees to move x-axis forward
        double x = Math.cos(degreesToR);
        double y = -Math.sin(degreesToR); // Flip sine/cosine to clockwise
        double rx = 0;
        double denominator = Math.max(Math.abs(y) + Math.abs(x), 1);
        double frontLeftPower = ((y + x) + rx / denominator) * power;
        double backLeftPower = ((y - x) + rx / denominator) * power;
        double frontRightPower = ((y - x) + rx / denominator) * power;
        double backRightPower = ((y + x) + rx / denominator) * power;
        double travelDistance = driveDistance(distance);

        //Track for front right instead of front left
        // *** NOTE: sine for the y axis has now been flipped and the quadrants corrected!
        DcMotor trackEncoderFor = robot.motorFrontLeft;
        String trackEncoderForTelemetry = "Front Left";
        if( degrees > 90 && degrees <= 180 || degrees < 360 && degrees >= 270  || degrees < -0 && degrees >= -90 ){
            trackEncoderFor = robot.motorFrontRight;
            trackEncoderForTelemetry = "Front Right";
        }
        //begin runtime for timeout and restart encoders
        runtime.reset();
        robot.restartEncoders();
        //while loop for active movement
        while (opModeIsActive() && (runtime.seconds() < timeout) && Math.abs(trackEncoderFor.getCurrentPosition()) < travelDistance) {
            //Set powers.
            robot.motorFrontLeft.setPower(frontLeftPower);
            robot.motorBackLeft.setPower(backLeftPower);
            robot.motorFrontRight.setPower(frontRightPower);
            robot.motorBackRight.setPower(backRightPower);
            //Telemetry.
            telemetry.addData("degrees",degrees);
            telemetry.addData("x",Math.abs(x));
            telemetry.addData("y",Math.abs(y));
            telemetry.addData("denominator", denominator);
            telemetry.addData("trackEncoderFor", trackEncoderForTelemetry);
            telemetry.addData("Track which motor?", trackEncoderFor.getCurrentPosition() );
            telemetry.addData("Travel Distance", travelDistance);
            telemetry.update();
        }
        robot.setPowers(0);
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
    //Reset IMU angle calculations.
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
    @param degrees Amount of degrees to be turned (0 to 359)
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
                power = pidRotate.performPID(getAngle());
                //power will be - on right turn.
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