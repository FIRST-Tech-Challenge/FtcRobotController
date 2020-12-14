package org.firstinspires.ftc.teamcode.competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.openftc.revextensions2.ExpansionHubEx;
import org.openftc.revextensions2.ExpansionHubMotor;
import org.openftc.revextensions2.RevBulkData;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static java.lang.Math.abs;


/**
 * This class is for setting up all the hardware components of the robot.
 * This will have all the sensors, motors and servos declarations.
 * It will also be used to initialize everything for autonomous
 * <p>
 * Note: 0, 0, 0 on the field is the robot in the blue depot with the intake facing the red depot
 */
public class Hardware {


    //Positions of the odometry wheels
    public ThreeTrackingWheelLocalizer odom = new ThreeTrackingWheelLocalizer(
            new ArrayList<>(Arrays.asList(
                    new Pose2d(4.25, 0, Math.PI / 2),
                    new Pose2d(0, 7.51, 0),
                    new Pose2d(0, -7.51, 0)))) {
        @Override
        public List<Double> getWheelPositions() {
            ArrayList<Double> wheelPositions = new ArrayList<>(3);
            wheelPositions.add(centerOdomTraveled);
            wheelPositions.add(leftOdomTraveled);
            wheelPositions.add(rightOdomTraveled);
            return wheelPositions;
        }
    };

    // Measurements and such kept as variables for ease of use
    // Ticks Per Rotation of an odometry wheel
    private static final double ODOM_TICKS_PER_ROTATION = 2048;
    // Radius of an odometry wheel in cm
    private static final double ODOM_WHEEL_RADIUS = 1.37795;
    // Circumference of an odometry wheel in cm
    private static final double WHEEL_CIRCUM = 2.0 * Math.PI * ODOM_WHEEL_RADIUS;
    // Number of ticks in a centimeter using dimensional analysis
    private static final double ODOM_TICKS_PER_CM = ODOM_TICKS_PER_ROTATION / (WHEEL_CIRCUM);


    //Distance from the center of the robot to the launch mechanism in inches.
    private static final double distCenterToLaunch = 7;
    //Gravitational constant used for calculating ring launch angle in inches per second squared
    private static final double ringGravitationalConstant = -386.09;
    //Radius of flyWheels in inches
    private static final double flyWheelRadius = 1.5;

    // Robot physical location]
    public double x, y, theta;

    // Map from hardware name to physical address
    private HardwareMap hwMap;

    // Gyro
    public BNO055IMU imu;

    // Drive train
    public DcMotorEx leftFront, rightFront, leftRear, rightRear;

    //Wobble Goal Lifter
    public Servo leftWobbleGoal, rightWobbleGoal;

    // Odometry hardware
    private DcMotorEx leftEncoder, rightEncoder, centerEncoder;

    // Rev Expansion Hub Data
    public RevBulkData bulkData;
    public ExpansionHubEx expansionHub;
    public ExpansionHubMotor leftOdom, rightOdom, centerOdom;

    // Odometry encoder positions
    public int leftEncoderPos, centerEncoderPos, rightEncoderPos;

    // Real world distance traveled by the wheels
    public double leftOdomTraveled, rightOdomTraveled, centerOdomTraveled;


    /**
     * Initialization of hardware
     *
     * @param mapping hardware map passed into class
     */
    public void init(HardwareMap mapping) {
        hwMap = mapping;

        // Drive train
        // left front
        leftFront = hwMap.get(DcMotorEx.class, "leftFront");
        // Motors don't have encoders on them because we're using odometry
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        // When motors aren't receiving power, they will attempt to hold their position
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // left rear
        leftRear = hwMap.get(DcMotorEx.class, "leftRear");
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // right front
        rightFront = hwMap.get(DcMotorEx.class, "rightFront");
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // right rear
        rightRear = hwMap.get(DcMotorEx.class, "rightRear");
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightRear.setDirection(DcMotor.Direction.REVERSE);
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Odometry encoder setup
        leftEncoder = hwMap.get(DcMotorEx.class, "leftFront");
        rightEncoder = hwMap.get(DcMotorEx.class, "rightFront");
        centerEncoder = hwMap.get(DcMotorEx.class, "rightRear");

        // Rev Expansions DLC
        expansionHub = hwMap.get(ExpansionHubEx.class, "Control Hub");
        leftOdom = (ExpansionHubMotor) hwMap.dcMotor.get("leftFront");
        rightOdom = (ExpansionHubMotor) hwMap.dcMotor.get("rightFront");
        centerOdom = (ExpansionHubMotor) hwMap.dcMotor.get("rightRear");

        //Wobble goal Servo setup
        leftWobbleGoal = hwMap.servo.get("leftWobbleGoal");
        rightWobbleGoal = hwMap.servo.get("rightWobbleGoal");


    }

    /**
     * Method for updating the position of the robot using roadrunner
     */
    public void updatePositionRoadRunner() {
        bulkData = expansionHub.getBulkInputData();

        // Change in the distance (centimeters) since the last update for each odometer
        double deltaLeftDist = -(getDeltaLeftTicks() / ODOM_TICKS_PER_CM);
        double deltaRightDist = -(getDeltaRightTicks() / ODOM_TICKS_PER_CM);
        double deltaCenterDist = -getDeltaCenterTicks() / ODOM_TICKS_PER_CM;

        // Update real world distance traveled by the odometry wheels, regardless of orientation
        leftOdomTraveled += deltaLeftDist;
        rightOdomTraveled += deltaRightDist;
        centerOdomTraveled += deltaCenterDist;

        odom.update();
        theta = odom.getPoseEstimate().component3();
        x = odom.getPoseEstimate().component1();
        y = odom.getPoseEstimate().component2();

        resetDeltaTicks();

    }

    /**
     * Resets the delta on all odometry encoders back to 0
     * */
    private void resetDeltaTicks() {
        leftEncoderPos = bulkData.getMotorCurrentPosition(leftOdom);
        rightEncoderPos = bulkData.getMotorCurrentPosition(rightOdom);
        centerEncoderPos = bulkData.getMotorCurrentPosition(centerOdom);
    }

    private int getDeltaLeftTicks() {
        return leftEncoderPos - bulkData.getMotorCurrentPosition(leftOdom);
    }

    private int getDeltaRightTicks() {
        return rightEncoderPos - bulkData.getMotorCurrentPosition(rightOdom);
    }

    private int getDeltaCenterTicks() {
        return centerEncoderPos - bulkData.getMotorCurrentPosition(centerOdom);
    }

    /**
     * Resets odometry position and values back to specific values
     *
     * @param x     X position to reset encoders to
     * @param y     Y position to reset encoders to
     * @param theta Rotational value to reset encoders to
     */
    public void resetOdometry(double x, double y, double theta) {
        odom.setPoseEstimate(new Pose2d(x, y, theta));

        leftOdomTraveled = 0;
        rightOdomTraveled = 0;
        leftOdomTraveled = 0;
        leftEncoderPos = 0;
        rightEncoderPos = 0;
        centerEncoderPos = 0;

        // Resets encoder values then sets them back to run without encoders because wheels and odometry are same pointer
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Drives the robot with the front being a specific direction of the robot
     *
     * @param forward  The forward value input (left stick y)
     * @param sideways The sideways value input (left stick x)
     * @param rotation The rotation value input (right stick x)
     */
    public void drive(double forward, double sideways, double rotation) {
        //adds all the inputs together to get the number to scale it by
        double scale = abs(rotation) + abs(forward) + abs(sideways);

        //scales the inputs when needed
        if (scale > 1) {
            forward /= scale;
            rotation /= scale;
            sideways /= scale;
        }
        //setting the motor powers to move
        leftFront.setPower(forward - rotation - sideways);
        leftRear.setPower(forward - rotation + sideways);
        rightFront.setPower(forward + rotation + sideways);
        rightRear.setPower(forward + rotation - sideways);
        //Left Front = +Speed + Turn - Strafe      Right Front = +Speed - Turn + Strafe
        //Left Rear  = +Speed + Turn + Strafe      Right Rear  = +Speed - Turn - Strafe
    }




    //Up and down left and right wobble goals
    /*

    WARNING CHANGE WOBBLE GOAL POSITIONS BEFORE RUNNING

     */
    public void leftWobbleGoalUp(){
        leftWobbleGoal.setPosition(1);
    }
    public void rightWobbleGoalUp(){
        rightWobbleGoal.setPosition(1);
    }
    public void leftWobbleGoalDown() {
        leftWobbleGoal.setPosition(0);
    }
    public void rightWobbleGoalDown(){
        rightWobbleGoal.setPosition(0);
    }
}