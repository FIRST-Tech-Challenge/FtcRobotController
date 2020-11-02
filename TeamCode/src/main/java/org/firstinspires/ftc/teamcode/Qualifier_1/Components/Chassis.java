package org.firstinspires.ftc.teamcode.Qualifier_1.Components;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Navigations.Odometry;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;
import static java.lang.Math.tan;

public class Chassis {
    //initialize motor
    DcMotorEx motorLeftFront;
    DcMotorEx motorRightFront;
    DcMotorEx motorLeftBack;
    DcMotorEx motorRightBack;

    // Initialize Encoder Variables
    final double robot_diameter = Math.sqrt(619.84);
    final double wheel_diameter = 3.93701;

    // these encoder variables vary depending on chassis type
    final double counts_per_motor_goBilda = 383.6;
    final double counts_per_inch = (counts_per_motor_goBilda*wheel_diameter * Math.PI)/54.48;  //2*(counts_per_motor_goBilda / (wheel_diameter * Math.PI))
    final double counts_per_degree = counts_per_inch * robot_diameter * Math.PI / 360;

    /* local OpMode members. */
    private LinearOpMode op             = null;
    private HardwareMap hardwareMap     = null;
    private ElapsedTime period          = new ElapsedTime();
    Odometry odom = new Odometry();

    BNO055IMU imu;
    Orientation             lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
    double IMUgain = 0.005;
    //set true to enable imu vice versa
    final boolean enableIMU = true;

    public Chassis() {

    }
    public void init(LinearOpMode opMode) {

        op = opMode;
        hardwareMap = op.hardwareMap;

        // Chassis motors
        motorLeftFront = (DcMotorEx) hardwareMap.dcMotor.get("motorLeftFront");
        motorRightFront = (DcMotorEx) hardwareMap.dcMotor.get("motorRightFront");
        motorLeftBack = (DcMotorEx) hardwareMap.dcMotor.get("motorLeftBack");
        motorRightBack = (DcMotorEx) hardwareMap.dcMotor.get("motorRightBack");

//        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
//
//        parameters.mode                = BNO055IMU.SensorMode.IMU;
//        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
//        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
//        parameters.loggingEnabled      = false;
//
//        imu = hardwareMap.get(BNO055IMU.class, "imu");
//        imu.initialize(parameters);
//        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
//
//        // make sure the imu gyro is calibrated before continuing.
//        while (!op.isStopRequested() && !imu.isGyroCalibrated())
//        {
//            op.sleep(50);
//            op.idle();
//        }
//
//        op.telemetry.addData("Mode", "waiting for start");
//        op.telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
//        op.telemetry.update();
//        op.sleep(500);

        // Chassis Motors
        motorLeftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorLeftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorRightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorLeftFront.setDirection(DcMotor.Direction.REVERSE);
        motorRightFront.setDirection(DcMotor.Direction.FORWARD);
        motorLeftBack.setDirection(DcMotor.Direction.REVERSE);
        motorRightBack.setDirection(DcMotor.Direction.FORWARD);

        // reset encoder count kept by left motor.
        motorLeftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        odom.init(opMode);
    }

    public void stopAllMotors() {
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
    }

    public void stopAllMotorsSideways() {
        motorLeftBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
        motorRightBack.setPower(0);
    }

    /*****Motor Movement*****/

    /******** Left Front Motor **********/
    public void moveMotorLeftFront(double distance){
        double ticksToMove = counts_per_inch * distance;
        double ticksLocationToMove = motorLeftFront.getCurrentPosition() + ticksToMove;
        motorLeftFront.setTargetPosition((int)ticksLocationToMove);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftFront.setPower(0.5);
        while (op.opModeIsActive() && motorLeftFront.isBusy())
        {
            op.telemetry.addData("leftFrontMotor ", motorLeftFront.getCurrentPosition() + " busy=" + motorLeftFront.isBusy());
            op.telemetry.update();
            op.idle();
        }
        //brake
        motorLeftFront.setPower(0);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /******** Right Front Motor **********/
    public void moveMotorRightFront(double distance){
        double ticksToMove = counts_per_inch * distance;
        double ticksLocationToMove = motorRightFront.getCurrentPosition() + ticksToMove;
        motorRightFront.setTargetPosition((int)ticksLocationToMove);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setPower(0.5);
        while (op.opModeIsActive() && motorRightFront.isBusy())
        {
            op.telemetry.addData("rightFrontMotor ", motorRightFront.getCurrentPosition() + " busy=" + motorRightFront.isBusy());
            op.telemetry.update();
            op.idle();
        }
        //brake
        motorRightFront.setPower(0);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    /******** Left Back Motor **********/
    public void moveMotorLeftBack(double distance){
        double ticksToMove = counts_per_inch * distance;
        double ticksLocationToMove = motorLeftBack.getCurrentPosition() + ticksToMove;
        motorLeftBack.setTargetPosition((int)ticksLocationToMove);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftBack.setPower(0.5);
        while (op.opModeIsActive() && motorLeftBack.isBusy())
        {
            op.telemetry.addData("leftBackMotor ", motorLeftBack.getCurrentPosition() + " busy=" + motorLeftBack.isBusy());
            op.telemetry.update();
            op.idle();
        }
        //brake
        motorLeftBack.setPower(0);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /******** Right Back Motor **********/
    public void moveMotorRightBack(double distance){
        double ticksToMove = counts_per_inch * distance;
        double ticksLocationToMove = motorRightBack.getCurrentPosition() + ticksToMove;
        motorRightBack.setTargetPosition((int)ticksLocationToMove);
        motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightBack.setPower(0.5);
        while (op.opModeIsActive() && motorRightBack.isBusy())
        {
            op.telemetry.addData("rightBackMotor", motorRightBack.getCurrentPosition() + " busy=" + motorRightBack.isBusy());
            op.telemetry.update();
            op.idle();
        }
        //brake
        motorRightBack.setPower(0);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void resetAngle()
    {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        globalAngle = 0;
    }

    public double getAngle()
    {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        //op.telemetry.addData("first angle: ", (int)angles.firstAngle);
        //op.telemetry.update();
        //op.sleep(1000);
        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle <= -180) //If the angle is -180, it should be 180, because they are at the same point. The acceptable angles are (-180, 180]
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    private double checkDirection()
    {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .1;

        angle = getAngle();

        /*if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;*/

        if (enableIMU == false) {
            correction = 0;
        } else {
            correction = angle * (-gain);
        }
        return correction;
    }

    /****Moving Robot****/

    public void moveForward(double distance, double power) {
        double ticksToMove = counts_per_inch * distance;
        double newLeftBackTargetPosition = motorLeftBack.getCurrentPosition() + ticksToMove;
        double newLeftFrontTargetPosition = motorLeftFront.getCurrentPosition() + ticksToMove;
        double newRightBackTargetPosition = motorRightBack.getCurrentPosition() + ticksToMove;
        double newRightFrontTargetPosition = motorRightFront.getCurrentPosition() + ticksToMove;
        motorLeftBack.setTargetPosition((int)newLeftBackTargetPosition);
        motorLeftFront.setTargetPosition((int)newLeftFrontTargetPosition);
        motorRightBack.setTargetPosition((int)newRightBackTargetPosition);
        motorRightFront.setTargetPosition((int)newRightFrontTargetPosition);

        op.telemetry.addData("ticks: ", (int)ticksToMove +
                "LB: " + (int)newLeftBackTargetPosition + "LF: " + (int)newLeftFrontTargetPosition +
                "RB: " + (int)newRightBackTargetPosition + "LB: " + (int)newRightFrontTargetPosition);
        op.telemetry.update();

        motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRightFront.setPower(power);
        motorLeftFront.setPower(power);
        motorRightBack.setPower(power);
        motorLeftBack.setPower(power);

        while (op.opModeIsActive() && (motorLeftBack.isBusy() && motorLeftFront.isBusy() && motorRightBack.isBusy() &&
                motorRightFront.isBusy()))
        {
            //correction = checkDirection();

            motorRightBack.setPower(power);
            motorRightFront.setPower(power);
            motorLeftBack.setPower(power);
            motorLeftFront.setPower(power);
//            op.telemetry.addData("correction", correction);
//            op.telemetry.update();
//            op.idle();
        }

        stopAllMotors();

        // Changes motor mode back to default
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveForwardIMU(double distance, double power) {
        double ticksToMove = counts_per_inch * distance;
        double newLeftBackTargetPosition = motorLeftBack.getCurrentPosition() + ticksToMove;
        double newLeftFrontTargetPosition = motorLeftFront.getCurrentPosition() + ticksToMove;
        double newRightBackTargetPosition = motorRightBack.getCurrentPosition() + ticksToMove;
        double newRightFrontTargetPosition = motorRightFront.getCurrentPosition() + ticksToMove;
        double currentPosition = 0;
        double deltaPosition = 0;
        double currentAngle = 0;
        double startingAngle = 0;

        startingAngle = getAngle();

        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        currentPosition = motorLeftBack.getCurrentPosition();
        deltaPosition = newLeftBackTargetPosition - currentPosition; //interchangable based on which way robot turns

        while (op.opModeIsActive() && (deltaPosition >= 0)) {
            currentPosition = motorLeftBack.getCurrentPosition();
            deltaPosition = newLeftBackTargetPosition - currentPosition;
            currentAngle = getAngle();
            correction = (currentAngle-startingAngle) * .01;//gain
            motorRightBack.setPower(power - correction);
            motorRightFront.setPower(power + correction);
            motorLeftBack.setPower(power - correction);
            motorLeftFront.setPower(power + correction);
            op.telemetry.addData("current pos", currentPosition + "delta pos", deltaPosition);
            op.telemetry.update();
            op.idle();
        }
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
    }

    public void moveBackward(double distance, double power) {
        moveForward(-distance, power);
    }

    public void moveBackwardIMU(double distance, double power) {
        moveForwardIMU(-distance, power);
    }

    public void moveRight(double distance, double power) {
        double ticksToMove = counts_per_inch * distance;
        double newLeftBackTargetPosition = motorLeftBack.getCurrentPosition() + ticksToMove;
        double newLeftFrontTargetPosition = motorLeftFront.getCurrentPosition() - ticksToMove;
        double newRightBackTargetPosition = motorRightBack.getCurrentPosition() - ticksToMove;
        double newRightFrontTargetPosition = motorRightFront.getCurrentPosition() + ticksToMove;
        motorLeftBack.setTargetPosition((int)newLeftBackTargetPosition);
        motorLeftFront.setTargetPosition((int)newLeftFrontTargetPosition);
        motorRightBack.setTargetPosition((int)newRightBackTargetPosition);
        motorRightFront.setTargetPosition((int)newRightFrontTargetPosition);

        motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        op.telemetry.addData("ticks: ", (int)ticksToMove +
                "LB: " + (int)newLeftBackTargetPosition + "LF: " + (int)newLeftFrontTargetPosition +
                "RB: " + (int)newRightBackTargetPosition + "LB: " + (int)newRightFrontTargetPosition);
        op.telemetry.update();

        motorLeftBack.setPower(power);
        motorRightBack.setPower(-power);
        motorLeftFront.setPower(-power);
        motorRightFront.setPower(power);

        while (op.opModeIsActive() && (motorLeftBack.isBusy() && motorLeftFront.isBusy() && motorRightBack.isBusy() &&
                motorRightFront.isBusy()))
        {

            motorRightBack.setPower(power - correction);
            motorRightFront.setPower(power + correction);
            motorLeftBack.setPower(power - correction);
            motorLeftFront.setPower(power + correction);
//            op.telemetry.addData("correction", correction);
//            op.telemetry.update();
//            op.idle();
        }

        stopAllMotorsSideways();

        // Changes motor mode back to default
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void moveRightIMU(double distance, double power, double startingAngle,double gain, double maxCorrection) {
        double ticksToMove = counts_per_inch * distance;
        double newLeftBackTargetPosition = motorLeftBack.getCurrentPosition() + ticksToMove;
        double currentPosition = 0;
        double deltaPosition = 0;
        double currentAngle = 0;
        double maxcorrection=0.16*power;

        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        currentPosition = motorLeftBack.getCurrentPosition();
        deltaPosition = ticksToMove;

        while (op.opModeIsActive() && (deltaPosition >= 0)) {
            currentPosition = motorLeftBack.getCurrentPosition();
            deltaPosition = newLeftBackTargetPosition - currentPosition;
            currentAngle = getAngle();
            correction = (currentAngle - startingAngle) * gain * power;//gain is 0.06 for power 0.85
            if (correction > maxcorrection) { correction = maxcorrection; }
            if (correction < -maxcorrection) { correction = -maxcorrection; }
            motorRightBack.setPower(-power - correction);
            motorRightFront.setPower(power - correction);
            motorLeftBack.setPower(power + correction);
            motorLeftFront.setPower(-power + correction);
        }

        stopAllMotorsSideways();

    }

    public void moveLeft(double distance, double power) {
        moveRight(-distance, power);
    }

    public void moveLeftIMU(double distance, double power, double startingAngle, double gain, double maxCorrection) {
        double ticksToMove = counts_per_inch * distance;
        double newLeftBackTargetPosition = motorLeftBack.getCurrentPosition() - ticksToMove;
        double currentPosition = 0;
        double deltaPosition = 0;
        double currentAngle = 0;
        double maxcorrection=0.16*power;

        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        currentPosition = motorLeftBack.getCurrentPosition();
        deltaPosition = ticksToMove;

        while (op.opModeIsActive() && (deltaPosition >= 0)) {
            currentPosition = motorLeftBack.getCurrentPosition();
            deltaPosition = currentPosition - newLeftBackTargetPosition ;
            currentAngle = getAngle();
            correction = (currentAngle-startingAngle) * gain*power;//gain=0.06 for power 0.85
            if (correction > maxcorrection) {correction = maxcorrection;}
            if (correction < -maxcorrection) {correction = -maxcorrection;}
            motorRightBack.setPower(power - correction);
            motorRightFront.setPower(-power - correction);
            motorLeftBack.setPower(-power + correction);
            motorLeftFront.setPower(power + correction);
        }
        stopAllMotorsSideways();
    }

    public void moveAngle2(double distance, double angle, double turn){
        //turn = counts_per_degree*turn;
        turn = 0; //TODO Fix the turn
        double powerLF = (1/Math.sqrt(2)) * (Math.sin(Math.toRadians(angle))+Math.cos(Math.toRadians(angle)));
        double powerLB = (1/Math.sqrt(2)) * (Math.cos(Math.toRadians(angle))-Math.sin(Math.toRadians(angle)));
        double ticksToMove = counts_per_inch*distance;
//        op.telemetry.addData("LB: ", powerLB);
//        op.telemetry.addData("LF: ", powerLF);
//        op.telemetry.addData("L: ", ticksToMove + turn);
//        op.telemetry.addData("R: ", ticksToMove - turn);
//        op.telemetry.update();
//        op.sleep(2000);
//        double newLeftBackTargetPosition = motorLeftBack.getCurrentPosition() + powerLB*(ticksToMove + turn);
//        double newLeftFrontTargetPosition = motorLeftFront.getCurrentPosition() + powerLF*(ticksToMove + turn);
//        double newRightBackTargetPosition = motorRightBack.getCurrentPosition() + powerLF*(ticksToMove - turn);
//        double newRightFrontTargetPosition = motorRightFront.getCurrentPosition() + powerLB*(ticksToMove - turn);
        double newLeftBackTargetPosition = motorLeftBack.getCurrentPosition() + powerLB*ticksToMove;
        double newLeftFrontTargetPosition = motorLeftFront.getCurrentPosition() + powerLF*ticksToMove;
        double newRightBackTargetPosition = motorRightBack.getCurrentPosition() + powerLF*ticksToMove;
        double newRightFrontTargetPosition = motorRightFront.getCurrentPosition() + powerLB*ticksToMove;
//        op.telemetry.addData("LB", "%.2f %.2f %.2f", (float) motorLeftBack.getCurrentPosition(), newLeftBackTargetPosition,powerLB*ticksToMove );
//        op.telemetry.addData("LF", "%.2f %.2f %.2f", (float) motorLeftFront.getCurrentPosition(), newLeftFrontTargetPosition, powerLF*ticksToMove);
//        op.telemetry.addData("RB", "%.2f %.2f %.2f", (float) motorRightBack.getCurrentPosition(), newRightBackTargetPosition, powerLF*ticksToMove);
//        op.telemetry.addData("RF", "%.2f %.2f %.2f", (float) motorRightFront.getCurrentPosition(), newRightFrontTargetPosition, powerLB*ticksToMove);
//        op.telemetry.update();
//        op.sleep(2000);
        motorLeftBack.setTargetPosition((int)newLeftBackTargetPosition);
        motorLeftFront.setTargetPosition((int)newLeftFrontTargetPosition);
        motorRightBack.setTargetPosition((int)newRightBackTargetPosition);
        motorRightFront.setTargetPosition((int)newRightFrontTargetPosition);

//        op.telemetry.addData("LB: ", powerLB);
//        op.telemetry.addData("LF: ", powerLF);
//        op.telemetry.update();

        motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRightFront.setPower(powerLB);
        motorLeftFront.setPower(powerLF);
        motorRightBack.setPower(powerLF);
        motorLeftBack.setPower(powerLB);

//        while (op.opModeIsActive() && (motorLeftBack.isBusy() && motorLeftFront.isBusy() && motorRightBack.isBusy() && motorRightFront.isBusy()))
//        while (op.opModeIsActive() && (motorLeftBack.isBusy() || motorLeftFront.isBusy() || motorRightBack.isBusy() || motorRightFront.isBusy()))
        while (op.opModeIsActive() && ((motorLeftBack.isBusy() && motorRightFront.isBusy()) || (motorRightBack.isBusy() && motorLeftFront.isBusy())))
        // Note: We used to have while loop going only when all 4 motors were busy. The above condition is modified for diagonal case when only 2 motors are busy.
        {
            op.telemetry.update();
        }

        stopAllMotors();

        newLeftBackTargetPosition = motorLeftBack.getCurrentPosition() + turn;
        newLeftFrontTargetPosition = motorLeftFront.getCurrentPosition() + turn;
        newRightBackTargetPosition = motorRightBack.getCurrentPosition() - turn;
        newRightFrontTargetPosition = motorRightFront.getCurrentPosition() -turn;
        motorLeftBack.setTargetPosition((int)newLeftBackTargetPosition);
        motorLeftFront.setTargetPosition((int)newLeftFrontTargetPosition);
        motorRightBack.setTargetPosition((int)newRightBackTargetPosition);
        motorRightFront.setTargetPosition((int)newRightFrontTargetPosition);

        motorRightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorRightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        motorRightFront.setPower(powerLB);
        motorLeftFront.setPower(powerLF);
        motorRightBack.setPower(powerLF);
        motorLeftBack.setPower(powerLB);

        stopAllMotors();

        // Changes motor mode back to default
        motorLeftFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorRightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void turnOdometry(double target, double power) {
        double currentAngle = odom.getAngle();
        int direction = 1;
        double difference = target - currentAngle;
        if (difference < 0) {
            direction = -1;
        }
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (op.opModeIsActive() && (difference >= 0.5)) {
            currentAngle = odom.getAngle();
            difference = target - currentAngle;
            if (difference * direction < 5) {
                power *= difference / 5;
                if(power<0.2){
                    power=0.2;
                }
            }
            motorRightBack.setPower(-power * direction);
            motorRightFront.setPower(-power * direction);
            motorLeftBack.setPower(power * direction);
            motorLeftFront.setPower(power * direction);
            op.telemetry.addData("current angle", currentAngle);
            op.telemetry.update();
            op.idle();
        }
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
    }

    public void moveForwardOdometry(double distance, double power) {
        double[] currentPosition = odom.track();
        double[] target_position = {0, 0, 0};
        double correction = 0;
        double anglecorrection = 0;
        int direction = 1;
        target_position[0] = currentPosition[0];
        target_position[1] = currentPosition[1] + distance;
        target_position[2] = currentPosition[2];
        double difference = target_position[1] - currentPosition[1];
        if (difference < 0) {
            direction = -1;
        }
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (op.opModeIsActive() && (difference >= 0.25)) {
            currentPosition = odom.track();
            difference = target_position[1] - currentPosition[1];
            correction = (currentPosition[0] - target_position[0]) * .005;//gain
            anglecorrection = (currentPosition[2]-target_position[2])*.005;
            if (difference * direction < 5) {
                power *= difference / 5;
                if(power<0.2){
                    power=0.2;
                }
            }
            motorRightBack.setPower(-power * direction - correction+anglecorrection);
            motorRightFront.setPower(-power * direction + correction+anglecorrection);
            motorLeftBack.setPower(-power * direction + correction-anglecorrection);
            motorLeftFront.setPower(-power * direction - correction-anglecorrection);
            op.telemetry.addData("current xpos", currentPosition[0] + "current ypos", currentPosition[1]);
            op.telemetry.update();
            op.idle();
        }
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
    }

    public void moveSideOdometry(double distance, double power) {//right is positive use distance to change direction
        double[] currentPosition = odom.track();
        double[] target_position = {0, 0, 0};
        double correction = 0;
        double anglecorrection=0;
        int direction = 1;
        target_position[0] = currentPosition[0] + distance;
        target_position[1] = currentPosition[1];
        target_position[2]= currentPosition[2];
        double difference = target_position[0] - currentPosition[0];
        if (difference < 0) {
            direction = -1;
        }
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (op.opModeIsActive() && (direction * difference >= 0.25)) {
            currentPosition = odom.track();
            difference = target_position[0] - currentPosition[0];
            correction = (currentPosition[1] - target_position[1]) * .005;//gain
            anglecorrection = (currentPosition[2]-target_position[2])*0.005;
            if (difference * direction < 5) {
                power *= difference / 5;
                if(power<0.2){
                    power=0.2;
                }
            }
            motorRightBack.setPower(-power * direction - correction-anglecorrection);
            motorRightFront.setPower(+power * direction - correction-anglecorrection);
            motorLeftBack.setPower(+power * direction - correction+anglecorrection);
            motorLeftFront.setPower(-power * direction - correction+anglecorrection);
            op.telemetry.addData("current xpos", currentPosition[0] + "current ypos", currentPosition[1]);
            op.telemetry.update();
            op.idle();
        }
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
    }


    public void xyPath(double x, double y, double power) {
        turnOdometry(0, power);
        moveForwardOdometry(y, power);
        moveSideOdometry(x, power);

    }

    public void StraightxyPath(double x, double y, double power) {
        double target_angle = atan2(y, x) * 180 / PI;
        turnOdometry(target_angle, power);
        moveForwardOdometry(sqrt(x * x + y * y), power);
    }

    public void StraightPointPath(double x, double y, double power) {
        double[] start_data = odom.track();
        y -= start_data[1];
        x -= start_data[0];
        double target_angle = atan2(y, x) * 180 / PI;
        turnOdometry(target_angle, power);
        moveForwardOdometry(sqrt(x * x + y * y), power);
    }

    public void DirectPointPath(double x, double y, double power) {
        double[] start_data = odom.track();
        double[] target = {atan2(y, x), x - start_data[0], y - start_data[1]};
        moveAngleOdometry(target[0], target[1], target[2], power);
    }

    public void DirectxyPath(double x, double y, double power) {
        double[] start_data = odom.track();
        double[] target = {atan2(y, x), x, y};
        moveAngleOdometry(target[0], target[1], target[2]-start_data[2] , power);
    }

    public void moveAngleOdometry(double angleInRadians, double x, double y, double power) {
        double[] currentPosition = odom.track();
        double[] startPosition = currentPosition;
        double[] target_position = {0, 0,0};
        int direction = 1;
        double[] misdirection = {0, 0};
        double[] gain = {0, 0};
        double anglecorrection;
        target_position[0] = currentPosition[0] + x;
        target_position[1] = currentPosition[1] + y;
        target_position[2] = currentPosition[2];
        double difference = sqrt((target_position[0] - currentPosition[0]) * (target_position[0] - currentPosition[0]) + (target_position[1] - currentPosition[1]) * (target_position[1] - currentPosition[1]));
        if (difference < 0) {
            direction = -1;
        }
        double[] anglePower = { sin(angleInRadians+PI/4), cos(angleInRadians+PI/4)};
        motorRightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorRightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorLeftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        while (op.opModeIsActive() && (difference >= 0.25)) {
            currentPosition = odom.track();
            difference = sqrt((target_position[0] - currentPosition[0]) * (target_position[0] - currentPosition[0]) + (target_position[1] - currentPosition[1]) * (target_position[1] - currentPosition[1]));
            if ((target_position[0] - currentPosition[0]) / (target_position[1] - currentPosition[1]) != tan(angleInRadians)) {
                if (target_position[1] - currentPosition[1] > target_position[0] - currentPosition[0]) {
                    misdirection[0] = (currentPosition[0]-startPosition[0]) * tan(angleInRadians) - (currentPosition[1]-startPosition[1]);//extra y needed
                    misdirection[0] *= sqrt(2) / 2;//convert to 45 degrees tilted
                    misdirection[1] = misdirection[0];
                } else {
                    misdirection[1] = (currentPosition[1]-startPosition[1]) / (tan(angleInRadians)) - (currentPosition[0]-startPosition[0]);//extra x needed
                    misdirection[0] *= sqrt(2) / 2;//convert to 45 degrees tilted
                    misdirection[1] = -misdirection[0];
                }
            }else{
                misdirection[0]=0;
                misdirection[1]=0;
            }
            if (difference * direction < 5) {
                power *= difference / 5;
            }

            gain[0] = 1 - misdirection[1] * .05;
            gain[1] = 1 - misdirection[0] * .05;
            if (difference * direction < 5) {
                power *= difference / 5;
            }
            if (power < 1) {
                gain[0] = 1 + misdirection[0] * .05;
                gain[1] = 1 + misdirection[1] * .05;
            }
            anglecorrection=currentPosition[2]-target_position[2];
            motorRightBack.setPower(power * anglePower[0]*gain[0]+anglecorrection);
            motorRightFront.setPower(power *anglePower[1]*gain[1]+anglecorrection);
            motorLeftBack.setPower(power *  anglePower[1]*gain[1]-anglecorrection);
            motorLeftFront.setPower(power * anglePower[0]*gain[0]-anglecorrection);
            op.telemetry.addData("current xpos", currentPosition[0] + "current ypos"+currentPosition[1]);
            op.telemetry.update();
            op.idle();
        }
        motorLeftBack.setPower(0);
        motorRightBack.setPower(0);
        motorLeftFront.setPower(0);
        motorRightFront.setPower(0);
    }
}