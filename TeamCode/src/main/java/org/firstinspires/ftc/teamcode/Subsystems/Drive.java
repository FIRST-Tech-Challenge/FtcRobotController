

package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Commands.Command;
import org.firstinspires.ftc.teamcode.Commands.DefaultCommands.DriveDefault;
import org.firstinspires.ftc.teamcode.Tools.Constants;
import org.firstinspires.ftc.teamcode.Tools.FinalPose;
import org.firstinspires.ftc.teamcode.Tools.Mouse;
import org.firstinspires.ftc.teamcode.Tools.PID;
import org.firstinspires.ftc.teamcode.Tools.Vector;

import org.json.JSONArray;
import org.json.JSONException;
import org.json.JSONObject;

public class Drive extends Subsystem {

    private DcMotorEx frontLeftMotor;
    private DcMotorEx backLeftMotor;
    private DcMotorEx frontRightMotor;
    public DcMotorEx backRightMotor;

    private final double TICKS_PER_REV = 2000;
    private final double WHEEL_DIAMETER = 0.048; // meters
    private final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;

    private double x = 0.0;
    private double y = 0.0;
    private double theta = 0.0;

    private double lastLeftPos = 0;
    private double lastRightPos = 0;
    private double lastCenterPos = 0;

    private final PID xPID = new PID(1, 0, 0);
    private final PID yPID = new PID(1, 0, 0.5);
    private final PID thetaPID = new PID(5, 0, 1);

    private long lastUpdateTime = 0;

    private double totalXTraveled = 0.0;
    private double totalYTraveled = 0.0;

    private final double L = 0.4064;
    private final double W = 0.4064;

    public Drive(String name, HardwareMap hardwareMap) {
        super(name);

        // Initialize motors using the HardwareMap
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "left_front");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "left_back");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "right_front");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "right_back");
        Mouse.init(hardwareMap);
// Set motor directions (if needed)
        frontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        backRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);

// Set zero power behavior for all motors
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

// Reset encoders
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

// Set to RUN_WITHOUT_ENCODER for odometry
        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }



    private void initialize(HardwareMap hardwareMap) {
        frontLeftMotor = hardwareMap.get(DcMotorEx.class, "left_front");
        backLeftMotor = hardwareMap.get(DcMotorEx.class, "left_back");
        frontRightMotor = hardwareMap.get(DcMotorEx.class, "right_front");
        backRightMotor = hardwareMap.get(DcMotorEx.class, "right_back");

        Mouse.init(hardwareMap);

        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        this.resetEncoder();
        lastUpdateTime = System.currentTimeMillis();
//        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public Vector purePursuitController(double currentX, double currentY, double currentTheta, int currentIndex,
                                        JSONArray pathPoints) throws JSONException {
        JSONObject targetPoint = pathPoints.getJSONObject(pathPoints.length() - 1);
        int targetIndex = pathPoints.length() - 1;
        for (int i = currentIndex; i < pathPoints.length(); i++) {
            JSONObject point = pathPoints.getJSONObject(i);
            double velocityMag = Math.sqrt((Math.pow(point.getDouble("x_velocity"), 2) + Math.pow(point.getDouble("y_velocity"), 2))
                    + Math.pow(point.getDouble("angular_velocity"), 2));
            double targetTheta = point.getDouble("angle");
            while (Math.abs(targetTheta - currentTheta) > Math.PI) {
                if (targetTheta - currentTheta > Math.PI) {
                    targetTheta -= 2 * Math.PI;
                } else if (targetTheta - currentTheta < -Math.PI) {
                    targetTheta += 2 * Math.PI;
                }
            }
            if (!insideRadius(currentX - point.getDouble("x") / Constants.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS,
                    currentY - point.getDouble("y") / Constants.AUTONOMOUS_LOOKAHEAD_LINEAR_RADIUS,
                    (currentTheta - targetTheta) / Constants.AUTONOMOUS_LOOKAHEAD_ANGULAR_RADIUS,
                    Constants.AUTONOMOUS_LOOKAHEAD_DISTANCE /* * velocityMag */ + 0.01)) {

                targetIndex = i;
                targetPoint = pathPoints.getJSONObject(i);
                break;
            }
        }

        double targetX = targetPoint.getDouble("x"), targetY = targetPoint.getDouble("y"),
                targetTheta = targetPoint.getDouble("angle");

        while (Math.abs(targetTheta - currentTheta) > Math.PI) {
            if (targetTheta - currentTheta > Math.PI) {
                targetTheta -= 2 * Math.PI;
            } else if (targetTheta - currentTheta < -Math.PI) {
                targetTheta += 2 * Math.PI;
            }
        }

        xPID.setSetPoint(targetX);
        yPID.setSetPoint(targetY);
        thetaPID.setSetPoint(targetTheta);

        xPID.updatePID(currentX);
        yPID.updatePID(currentY);
        thetaPID.updatePID(currentTheta);

        double xVelNoFF = xPID.getResult();
        double yVelNoFF = yPID.getResult();
        double thetaVelNoFF = -thetaPID.getResult();

        double feedForwardX = targetPoint.getDouble("x_velocity") / 2;
        double feedForwardY = targetPoint.getDouble("y_velocity") / 2;
        double feedForwardTheta = -targetPoint.getDouble("angular_velocity") / 2;

        return new Vector(feedForwardX + xVelNoFF, -(feedForwardY + yVelNoFF));
    }

    public void teleopDrive(Gamepad gamepad1) {
        double x = -gamepad1.left_stick_x*2;
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;



        double frontLeftPower = (-y + x + rx);
        double backLeftPower = (y + x - rx);
        double frontRightPower = (y + x + rx);
        double backRightPower = (y - x + rx);


        frontLeftMotor.setPower(frontLeftPower);
        backLeftMotor.setPower(backLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backRightMotor.setPower(backRightPower);
    }

    public  void stop() {
        drive(0,0,0,0);

        backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    }

    public void drive(double leftFrontPower, double rightFrontPower, double leftBackPower, double rightBackPower) {
        frontLeftMotor.setPower(-leftFrontPower);
        frontRightMotor.setPower(-rightFrontPower);
        backLeftMotor.setPower(-leftBackPower);
        backRightMotor.setPower(-rightBackPower);
    }



    public void FeildCentric(Gamepad gamepad) {




        double x = -gamepad.left_stick_x*2;
        double y = -gamepad.left_stick_y;
        double rx = -gamepad.right_stick_x;


        double botHeading = -Math.toRadians(Mouse.getTheta());
        Mouse.update();

        if (gamepad.options) {
            gamepad.setLedColor(255,0,0, 1000);
            Mouse.configureOtos();
        }




        double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
        double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

        double frontLeftPower = (-rotY + rotX + rx);
        double backLeftPower = (-rotY - rotX + rx);
        double frontRightPower = (-rotY - rotX - rx);
        double backRightPower = (rotY - rotX + rx);



        frontLeftMotor.setPower(-frontLeftPower);
        backLeftMotor.setPower(-backLeftPower);
        frontRightMotor.setPower(-frontRightPower);
        backRightMotor.setPower(-backRightPower);




    }

    public void resetEncoder() {
        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        x = 0.0;
        y = 0.0;
        theta = 0.0;

        lastLeftPos = 0;
        lastRightPos = 0;
        lastCenterPos = 0;
    }



    private boolean insideRadius(double deltaX, double deltaY, double deltaTheta, double radius) {
        return Math.sqrt(Math.pow(deltaX, 2) + Math.pow(deltaY, 2) + Math.pow(deltaTheta, 2)) < radius;
    }

    public  double getVelocityBackLeft() {
        return backLeftMotor.getVelocity();
    }

    public  double getVelocityBackRight() {
        return backRightMotor.getVelocity();
    }

    public  double getVelocityFrontLeft() {
        return frontLeftMotor.getVelocity();
    }

    public  double getVelocityFrontRight() {
        return frontRightMotor.getVelocity();
    }

    public  double direction() {
        return direction();
    }


    public  void update() {
        double imuTheta = Peripherals.getYawDegrees();

        double currentLeftPos = getLeftEncoder();
        double currentRightPos = getRightEncoder();
        double currentCenterPos = getCenterEncoder();

        double deltaLeft = currentLeftPos - lastLeftPos;
        double deltaRight = currentRightPos - lastRightPos;
        double deltaCenter = currentCenterPos - lastCenterPos;

        lastLeftPos = currentLeftPos;
        lastRightPos = currentRightPos;
        lastCenterPos = currentCenterPos;

        double distanceLeft = deltaLeft * WHEEL_CIRCUMFERENCE / TICKS_PER_REV;
        double distanceRight = deltaRight * WHEEL_CIRCUMFERENCE / TICKS_PER_REV;
        double distanceCenter = deltaCenter * WHEEL_CIRCUMFERENCE / TICKS_PER_REV;

        double avgForwardMovement = (distanceLeft + distanceRight) / 2.0;

        double deltaTheta = Math.toRadians(imuTheta - theta);
        theta = imuTheta;

        double thetaRadians = Math.toRadians(theta);

        double deltaX = avgForwardMovement * Math.cos(thetaRadians) - distanceCenter * Math.sin(thetaRadians);
        double deltaY = avgForwardMovement * Math.sin(thetaRadians) + distanceCenter * Math.cos(thetaRadians);

        x += deltaX;
        y += deltaY;

        x = Math.round(x * 1000) / 1000.0;
        y = Math.round(y * 1000) / 1000.0;
    }


    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }


    public double getLastLeftPos(){
        return lastLeftPos;
    }

    public double getLastRightPos(){
        return lastRightPos;
    }

    public double getLastCenterPos(){
        return lastCenterPos;
    }

    public double getOdometryX() {
        return x;
    }

    public double getOdometryY() {
        return y;
    }

    public double getOdometryTheta() {
        return theta;
    }

    public double getTotalXTraveled() {
        return totalXTraveled;
    }

    public double getTotalYTraveled() {
        return totalYTraveled;
    }

    public void setPosition(double fieldX, double fieldY, double fieldTheta) {
        x = fieldX;
        y = fieldY;
        theta = fieldTheta;

        lastLeftPos = getLeftEncoder();
        lastRightPos = getRightEncoder();
        lastCenterPos = getCenterEncoder();
    }

    public int getLeftEncoder() {
        return backRightMotor.getCurrentPosition();
    }
    public  int getRightEncoder() {
        return frontLeftMotor.getCurrentPosition();
    }
    public  int getBackEncoder(){
        return backLeftMotor.getCurrentPosition();
    }

 /*   public static void moveToAprilTag(int ID){
            double tagX = FieldOfMerit.x;
            double tagY = FieldOfMerit.y;
            double tagTheta = FieldOfMerit.tagyaw;
            PID turnPID = new PID(1, 0, 0);
            turnPID.setSetPoint(Peripherals.getYawDegrees() -Math.toDegrees(tagTheta));
            turnPID.updatePID(Peripherals.getYawDegrees());
            if (!(turnPID.getError() == 0)) {
                if (Math.abs(turnPID.getError()) < 2) {
                    double distance = Math.sqrt(x * x + y * y);
                    if ((distance < 0.1)){
                        DriveCommand.drive(0.5,0.5,0.5,0.5);
                    }else {
                        DriveCommand.stop();
                    }
                }else {
                    double power = turnPID.getResult();
                    DriveCommand.drive(power, -power, power, -power);
                }

        }
    }*/

    public  int getCenterEncoder() {
        return frontRightMotor.getCurrentPosition();
    }

    public void autoDrive(Vector vector, double angle) {
        double vx = vector.getI();
        double vy = -vector.getJ();

        double rotationFactor = -(angle);

        double botHeading = Math.toRadians(FinalPose.Yaw);

        double rotX = - vx * Math.sin(botHeading) + vy * Math.cos(botHeading);
        double rotY = vx * Math.cos(botHeading) + vy * Math.sin(botHeading);

//        rotX *= 1.1;

        double denominator = Math.max(0.3, Math.abs(rotY) + Math.abs(rotX) + Math.abs(rotationFactor));
        double frontLeftPower = (-rotX + rotY - rotationFactor) / 4;
        double frontRightPower = (-rotX - rotY + rotationFactor) / 2;
        double backLeftPower = (rotX - rotY - rotationFactor) / 4;
        double backRightPower = (-rotX + rotY + rotationFactor) / 2;

/*
        double frontLeftPower = vx + vy + rotationFactor;
        double frontRightPower = vx - vy - rotationFactor;
        double backLeftPower = vx - vy + rotationFactor;
        double backRightPower = vx + vy - rotationFactor;

        double maxMagnitude = Math.max(Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower)),
                Math.max(Math.abs(backLeftPower), Math.abs(backRightPower)));

        if (maxMagnitude > 1) {
            frontLeftPower /= maxMagnitude;
            frontRightPower /= maxMagnitude;
            backLeftPower /= maxMagnitude;
            backRightPower /= maxMagnitude;
        }
*/

        drive(frontLeftPower, frontRightPower, -backLeftPower, -backRightPower);
    }

    public void sketchDrive(Gamepad gamepad1) {
        if (gamepad1.dpad_up) {
            drive(1,  -1, 1, 1);
        }
//        } else if (gamepad1.dpad_right) {
//            drive(1,1,1,1);
//        } else if (gamepad1.dpad_left) {
//            drive(-1,-1,-1,-1);
//        } else if (gamepad1.dpad_down) {
//            drive(1,-1,1,-1);
//        } else {
        stop();
        drive(0,0,0,0);
    }




    public double leftFrontPos(){
        return frontLeftMotor.getCurrentPosition();
    }
    public  double RightFrontPos(){
        return frontRightMotor.getCurrentPosition();
    }
    public  double leftBackPos(){
        return backLeftMotor.getCurrentPosition();
    }
    public  double RightBackPos(){
        return backRightMotor.getCurrentPosition();
    }

    @Override
    public void setDefaultCommand(Command command) {
        super.setDefaultCommand(command);
    }

    @Override
    public Command getDefaultCommand() {
        return new DriveDefault(this); // Retrieve the set default command
    }
}