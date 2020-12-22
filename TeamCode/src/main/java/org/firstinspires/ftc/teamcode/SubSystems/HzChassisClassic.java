package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

/** Definition of Robot Chassis. <BR>
 *  Chassis has : <BR>
 *      4 DC motors connected to Mecanum wheels <BR>
 *
 *      Chassis : New robot : 5202 Series Yellow Jacket Planetary Gear Motor (13.7:1 Ratio, 435 RPM, 3.3 - 5V Encoder) <BR>
 *         Encoder Countable Events Per Revolution (Output Shaft)	383.6 (Rises & Falls of Ch A & B) <BR>
 *
 *      Test Robot : 5202 Series Yellow Jacket Planetary Gear Motor (19.2:1 Ratio, 312 RPM, 3.3 - 5V Encoder)
 *         Encoder Countable Events Per Revolution (Output Shaft)	537.6 (Rises & Falls of Ch A & B) <BR>
 *
 *
 * @ChassisMethods : Chassis(HardwareMap) - Constructor
 * @ChassisMethods : initChassis()
 * @ChassisMethods : configureChassis()
 * @ChassisMethods : resetChassis()
 * @ChassisTeleOpMethods : runByGamepadCommand()
 * @ChassisAutoMethods : runDistance()
 * @ChassisAutoMethods : turnRobotByAngle()
 */

public class HzChassisClassic {

    //Declare Chassis Motor and configurations
    public DcMotor frontLeft;
    public DcMotor frontRight;
    public DcMotor backLeft;
    public DcMotor backRight;

    //Declare Chassis Configuration variables
    public double wheelRadius;
    public double robotRadius;
    public double target90degRotations;

    //Timer for timing out an action if needed
    ElapsedTime ChassisMotionTimeOut = new ElapsedTime();

    public boolean configureRobot = false;

    public double ChassisMotorEncoderCount = 537.6;  // for 5202 19.2:1 Ratio, 312 RPM
                                                     // 383.6 for 5202 13.7:1 Ratio, 435 RPM motor

    /**
     * Constructor of Chassis. <BR>
     * Sets hardwareMap on hub1 with 4 motors <BR>
     * Configures Robot for size and mecanum wheel directions <BR>
     * Initialize Robot to right component modes. <BR>
     * @param hardwareMap HardwareMap to be setup on Hub1
     */
    public HzChassisClassic(HardwareMap hardwareMap) {
        //Map DCMotors from configuration
        frontLeft = hardwareMap.dcMotor.get("flmotor");
        frontRight = hardwareMap.dcMotor.get("frmotor");
        backLeft = hardwareMap.dcMotor.get("blmotor");
        backRight = hardwareMap.dcMotor.get("brmotor");

        //Configure Robot to dimensions and modified for wheel type
        configureRobot();
    }

    /**
     * Configure Chassis for size and mecanum wheel directions
     */
    public void configureRobot(){
        wheelRadius = 1.965*7/5; //100mm and applied correction factor of 7/5 for slippage <TO-BE-UPDATED>
        robotRadius = 8.54*7/5; //Was 8.64 Radius = half of longest diagonal = 0.5*sqrt(sq(14.5)+sq(10.5).and applied correction factor of 7/5 for slippage
        //Set direction of motors wrt motor drive set up, so that wheels go forward +y power
        //<TO-BE-UPDATED>

        target90degRotations = (Math.PI*robotRadius/2)/(2*Math.PI*wheelRadius);

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        configureRobot = true;
    }


    /**
     * Initialize Chassis to right component modes - Reset, Set Zero Behavior
     */
    public void initChassis() {
        resetChassis();
        setZeroBehavior(DcMotor.ZeroPowerBehavior.FLOAT); // To avoid jerk at start
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Reset function for motor encoders to be set to reset state of encoder. <BR>
     * Usage of this is typically followed by using setZeroBehaviour and then setting
     * the mode for the motor <BR>
     *
     * Reset Color Sensors to off for TeleOpMode
     */
    public void resetChassis() {

        DcMotor.RunMode runMode = frontLeft.getMode();
        frontLeft.setTargetPosition(0);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(runMode);

        runMode = frontRight.getMode();
        frontRight.setTargetPosition(0);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(runMode);

        runMode = backLeft.getMode();
        backLeft.setTargetPosition(0);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(runMode);

        runMode = backRight.getMode();
        backRight.setTargetPosition(0);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(runMode);

    }

    /**
     * Function to set the behaviour of the motor on passing Zero power to the motor <BR>
     * @param zeroPowerBehavior could be BRAKE or FLOAT. When not defined, it is set
     *                          to UNKNOWN state, which is not desired.
     */
    public void setZeroBehavior(DcMotor.ZeroPowerBehavior zeroPowerBehavior) {
        frontLeft.setZeroPowerBehavior(zeroPowerBehavior);
        frontRight.setZeroPowerBehavior(zeroPowerBehavior);
        backLeft.setZeroPowerBehavior(zeroPowerBehavior);
        backRight.setZeroPowerBehavior(zeroPowerBehavior);
    }

    /**
     * Set the mode of the DC motor to RUN_WITHOUT_ENCODER (run at achievable velocity),
     * RUN_USING_ENCODER (run at a targeted velocity) or RUN_TO_POSITION (PID based rotation to
     * achieve the desited encoder count)
     * @param runMode RUN_WITHOUT_ENCODER
     */
    public void setMotorMode(DcMotor.RunMode runMode) {
        frontLeft.setMode(runMode);
        frontRight.setMode(runMode);
        backLeft.setMode(runMode);
        backRight.setMode(runMode);
    }

        /**
     * Method to move chassis based on computed vector inputs from Gamepad Joystick inputs
     * @param targetAngle targetAngle = Math.atan2(leftStickY, leftStickX)
     * @param turn turn = rightStickX
     * @param power power = Math.hypot(leftStickX, leftStickY)
     */
    public void runByGamepadCommand(double targetAngle, double turn, double power) {
        //Rotate angle by 45 degrees to align to diagonal angles on mecannum wheel setup
        double turnAngle = targetAngle - Math.PI / 4;

        //Distribute power to wheels a cos and sin of vector.
        // Add turn as input from right stick to add in radiants
        frontLeft.setPower(power * Math.cos(turnAngle) + turn);
        frontRight.setPower(power * Math.sin(turnAngle) - turn);
        backLeft.setPower(power * Math.sin(turnAngle) + turn);
        backRight.setPower(power * Math.cos(turnAngle) - turn);
        setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    /* Method to move chassis based on computed vector inputs for a set distance.
     * To be used in Autonomous mode for moving by distance or turning by angle.
     * Uses PID loop in motors to ensure motion without errors
     *
     * @param distance +ve for forward, -ve for backward
     * @param strafeDirection 0 for forward or backward, 1 for right, -1 for left
     * @param power to run motors
     * @param callingOpMode passed for checking for isStopRequested()
     */
    public void runFwdBackLeftRight(
            double distance,
            double strafeDirection,
            double power,
            LinearOpMode callingOpMode){
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);
        resetChassis();

        //Max Total Rotations of wheel = distance / circumference of wheel
        double targetRotations = distance/(2*Math.PI*wheelRadius);

        //set fwdbackdirection, +ve for forward and negative for backward
        double fwdbackdirection = distance /Math.abs(distance);

        while (!callingOpMode.isStopRequested() &&
                (Math.abs(backLeft.getCurrentPosition()) < Math.abs(ChassisMotorEncoderCount * targetRotations)
                )
              ){
            if(strafeDirection == 0) {
                //Go forward or backward
                frontLeft.setPower(fwdbackdirection*power);
                frontRight.setPower(fwdbackdirection*power);
                backLeft.setPower(fwdbackdirection*power);
                backRight.setPower(fwdbackdirection*power);
            } else {
                frontLeft.setPower(strafeDirection* power);
                frontRight.setPower(-strafeDirection* power);
                backLeft.setPower(-strafeDirection* power);
                backRight.setPower(strafeDirection* power);
            }
        }
        setZeroBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //#TOBECHECKED TO AVOID JERK
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }

    /**
     * Method to turn robot by 90 degrees
     * @param clockOrAntiClockwise + 1 for clockwise, -1 for anticlockwise
     * @param power to run motors
     * @param callingOpMode passed for checking for isStopRequested()
     */
    public void turnby90degree(
            int clockOrAntiClockwise,
            double power,
            LinearOpMode callingOpMode){
        resetChassis();
        setMotorMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (!callingOpMode.isStopRequested() &&
                (Math.abs(backLeft.getCurrentPosition()) < Math.abs(ChassisMotorEncoderCount * target90degRotations))
              ) {
            frontLeft.setPower(clockOrAntiClockwise*power);
            frontRight.setPower(-clockOrAntiClockwise*power);
            backLeft.setPower(clockOrAntiClockwise*power);
            backRight.setPower(-clockOrAntiClockwise*power);
        }
        frontLeft.setPower(0.0);
        frontRight.setPower(0.0);
        backLeft.setPower(0.0);
        backRight.setPower(0.0);
    }
}
