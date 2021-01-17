package org.firstinspires.ftc.teamcode.competition;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.helperclasses.ThreadPool;
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

    public enum SelectedGoal
    {

        HIGHGOAL,
        MIDGOAL,
        LOWGOAL,
        POWERSHOTONE,
        POWERSHOTTWO,
        POWERSHOTTHREE

    }

    //Positions of the odometry wheels
    public ThreeTrackingWheelLocalizer odom = new ThreeTrackingWheelLocalizer(
            new ArrayList<>(Arrays.asList(
                    new Pose2d(-4.58, 0, -Math.PI / 2),
                    new Pose2d(0, 6.485, 0),
                    new Pose2d(0, -6.485, 0)))) {
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
    private static final double ODOM_TICKS_PER_ROTATION = 2048*4;
    // Radius of an odometry wheel in cm
    private static final double ODOM_WHEEL_RADIUS = 0.688975;
    // Circumference of an odometry wheel in cm
    private static final double WHEEL_CIRCUM = 2.0 * Math.PI * ODOM_WHEEL_RADIUS;
    // Number of ticks in a centimeter using dimensional analysis
    private static final double ODOM_TICKS_PER_IN = ODOM_TICKS_PER_ROTATION / (WHEEL_CIRCUM);


    //Distance from the center of the t to the launch mechanism in inches.
    public static final double distCenterToLaunch = 8;
    //Gravitational constant used for calculating ring launch angle in inches per second squared
    private static final double ringGravitationalConstant = 386.09;
    //Radius of flyWheels in inches
    private static final double flyWheelRadius = 1.5;

    // Robot physical location
    public static double x, y, theta;

    //Robot velocity
    public double xVelocity, yVelocity, thetaVelocity;

    public ElapsedTime e;

    // Map from hardware name to physical address
    private HardwareMap hwMap;

    // Gyro
    public BNO055IMU imu;

    // Drive train
    public DcMotorEx leftFront, rightFront, leftRear, rightRear;


    //Wobble Goal Lifter
    public Servo leftWobbleGoal, rightWobbleGoal;

    //Intake
    DcMotor intakeMotor;

    //flywheelRotateServo
    public Servo flywheelRotateServoLeft;

    //claw servos
    public Servo clawServoLeft;
    public Servo clawServoRight;

    // Odometry hardware
    private DcMotorEx leftEncoder, rightEncoder, centerEncoder;

    // Rev Expansion Hub Data
    public RevBulkData bulkData;
    public ExpansionHubEx expansionHub;
    public ExpansionHubMotor leftOdom, rightOdom, centerOdom;

    //Fly wheels
    public DcMotorEx flywheelMotorLeft;
    public DcMotorEx flywheelMotorRight;

    //Servo to move rind from magazine into flywheels
    public Servo flicker;

    private boolean isFlickerMoving;
    public byte queuedFlicks = 0;

    // Odometry encoder positions
    public int leftEncoderPos, centerEncoderPos, rightEncoderPos;

    // Real world distance traveled by the wheels
    public double leftOdomTraveled, rightOdomTraveled, centerOdomTraveled;

    double lastTime = 0;


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
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        // left rear
        leftRear = hwMap.get(DcMotorEx.class, "leftRear");
        leftRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        // right front
        rightFront = hwMap.get(DcMotorEx.class, "rightFront");
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // right rear
        rightRear = hwMap.get(DcMotorEx.class, "rightRear");
        rightRear.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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


        //Flywheels
        flywheelMotorLeft = hwMap.get(DcMotorEx.class,"flywheelMotorLeft");
        flywheelMotorRight = hwMap.get(DcMotorEx.class,"flywheelMotorRight");
        flywheelMotorLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //Wobble goal Servo setup
        //leftWobbleGoal = hwMap.servo.get("leftWobbleGoal");
        //rightWobbleGoal = hwMap.servo.get("rightWobbleGoal");

        //Intake
        intakeMotor = hwMap.dcMotor.get("intake");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        //flywheel rotating
        flywheelRotateServoLeft = hwMap.servo.get("flywheelRotateServoLeft");
        flywheelRotateServoLeft.setPosition(.9);

        //claw servos
        //clawServoLeft = hwMap.servo.get("clawServoLeft");
        //clawServoRight = hwMap.servo.get("clawServoRight");

        flicker = hwMap.servo.get("flicker");

        e = new ElapsedTime();
        e.startTime();

    }

    public void initServos()
    {

        flicker.setPosition(1);

    }

    /**
     * Method for updating the position of the robot using roadrunner
     */
    public void updatePositionRoadRunner() {
        try
        {
            bulkData = expansionHub.getBulkInputData();
        }catch(Exception e)
        {

            return;

        }

        // Change in the distance (centimeters) since the last update for each odometer
        double deltaLeftDist = -(getDeltaLeftTicks() / ODOM_TICKS_PER_IN);
        double deltaRightDist = -(getDeltaRightTicks() / ODOM_TICKS_PER_IN);
        double deltaCenterDist = -getDeltaCenterTicks() / ODOM_TICKS_PER_IN;

        // Update real world distance traveled by the odometry wheels, regardless of orientation

        leftOdomTraveled += deltaLeftDist*1.01;
        rightOdomTraveled += deltaRightDist;
        centerOdomTraveled += deltaCenterDist;
        double lastX = x;
        double lastY = y;
        double lastTheta = theta;
        double thisTime = e.seconds();
        odom.update();
        theta = odom.getPoseEstimate().component3();
        x = odom.getPoseEstimate().component1();
        y = odom.getPoseEstimate().component2();
        if(thisTime-lastTime>.15)
        {

            xVelocity = (x - lastX ) / (thisTime-lastTime);
            yVelocity = (y - lastY) / (thisTime-lastTime);
            thetaVelocity = (lastTheta - theta) / (thisTime-lastTime);
            lastTime = thisTime;

        }

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
     * Sets the power of the fly wheels
     * @Param power power to run the fly wheels at
     * */
    public void setFlyWheelPower(double power)
    {

        flywheelMotorLeft.setPower(power);
        flywheelMotorRight.setPower(power);

    }

    /**
     * Sets the velocity of the fly wheels
     * @Param velocity velocity to run the fly wheels at in RPM (RPM of the motor with not including gearing)
     * */
    public void setFlyWheelVelocity(double velocity)
    {

        flywheelMotorLeft.setVelocity(velocity);
        flywheelMotorRight.setVelocity(velocity);

    }


     /** Sets the power of the intake motor to a given power
     * @param power motor power to which the intake motor will be set
     * */
    public void setIntakePower(double power)
    {

        intakeMotor.setPower(power);

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

    /**
    * Raise the left wobble goal grabber
    */
    public void leftWobbleGoalUp()
    {
        leftWobbleGoal.setPosition(1);
    }

    /**
     * Raises the right wobble goal grabber
     */
    public void rightWobbleGoalUp()
    {
        rightWobbleGoal.setPosition(1);
    }

    /**
     * Lowers left wobble goal grabber
     */
    public void leftWobbleGoalDown()
    {
        leftWobbleGoal.setPosition(0);
    }

    /**
     * Lowers right wobble goal grabber
     */
    public void rightWobbleGoalDown()
    {
        rightWobbleGoal.setPosition(0);
    }

    /*
    //raises left claw
    public void clawServoLeftUp() {clawServoLeft.setPosition(1);}

    //lowers left claw
    public void clawServoLeftDown() {clawServoLeft.setPosition(0);}

    //raises right claw
    public void clawServoRightUp() {clawServoRight.setPosition(1);}

    //lowers right claw
    public void clawServoRightDown() {clawServoLeft.setPosition(0);}
    */



    public void flickRing()
    {

        if(!isFlickerMoving)
        {

            isFlickerMoving=true;
            flicker.setPosition(0);
            Thread wait = new Thread()
            {

                @Override
                public void run()
                {

                    ElapsedTime e = new ElapsedTime();
                    e.startTime();
                    while(e.milliseconds()<75);
                    flicker.setPosition(1);
                    e = new ElapsedTime();
                    e.startTime();
                    while(e.milliseconds()<200);
                    isFlickerMoving=false;
                    if(queuedFlicks>0)
                    {
                        queuedFlicks--;
                        flickRing();
                    }

                }

            };
            ThreadPool.pool.submit(wait);


        }
        else
        {

            if(queuedFlicks<2)
                queuedFlicks++;

        }


    }


    /**
     * Calculates an angle to launch rings at so they will hit a specified game target (assuming the robot turns to face the target)
     *
     * @param goal which of the targets to aim rings at
     * @return An array containing two double. The first double being angle which, if rings are launched at with the correct velocity when facing a target, will hit that target at the top of their parabola, and the second double being said velocity.
     * */
    public double[] getFlyWheelAngle(SelectedGoal goal)
    {

        //set the x, y, and z of the target to shoot rings at
        double zGoal=0;
        double xGoal=0;
        double yGoal=0;
        switch(goal)
        {

            case LOWGOAL:
                xGoal=-144;
                yGoal=-36;
                zGoal=17;
                break;
            case MIDGOAL:
                xGoal=-144;
                yGoal=-36;
                zGoal=27.0625;
                break;
            case HIGHGOAL:
                xGoal=-144;
                yGoal=-36;
                zGoal=35.875;
                break;
            case POWERSHOTONE:
                xGoal=-144;
                yGoal=-54;
                zGoal=30.875;
                break;
            case POWERSHOTTWO:
                xGoal=-144;
                yGoal=-61.5;
                zGoal=30.875;
                break;
            case POWERSHOTTHREE:
                x=-144;
                yGoal=-69;
                zGoal=30.875;
                break;

        }

        //calculate what the distance from the launch mech to the goal will be after the robot has turned to face the goal
        double distance = Math.sqrt(Math.pow(x-xGoal,2)+Math.pow(y-yGoal,2))-distCenterToLaunch;
        //return the angle to launch rings
        double angle = Math.atan(2*zGoal/distance);
        double velocity = distance/Math.cos(angle)*Math.sqrt(ringGravitationalConstant/(2*(distance*Math.tan(angle)-zGoal)));

        return new double[]{angle,velocity};

    }

}