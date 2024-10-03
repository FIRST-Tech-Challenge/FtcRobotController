package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Bot {

    //OpMode Declaration
    private LinearOpMode opMode;

    //Motor Declaration
    private DcMotor leftMotorFront;
    private DcMotor rightMotorFront;
    private DcMotor leftMotorBack;
    private DcMotor rightMotorBack;

    private DcMotor rightLift;
    private DcMotor leftLift;

    private CRServo topIntake;
    private CRServo bottomIntake;

    //extend and pivot
    private DcMotor extendArmMotor;
    private DcMotor armPivotMotor;


    //Statistics for measurements
    static final double WHEEL_DIAMETER_INCHES = 1; // For circumference / distance measurements
    private static final int TICKS_PER_REV = 1440;
    private static final double ARM_GEAR_RATIO = 1.0;
    private static final double DISTANCE_PER_REV = 10.0;

    private ElapsedTime runtime = new ElapsedTime();

    /**
     * Constructor for Bot object
     * @param opMode
     */
    public Bot(LinearOpMode opMode) {
        this.opMode = opMode;
        init(opMode.hardwareMap);
    }

    /**
     * Initialize hardware mapping for robot
     * @param map
     */
    public void init(HardwareMap map){

        //Connecting declared motors to classes of DcMotors and respected names
        leftMotorFront = map.get(DcMotor.class, "left_front");
        leftMotorBack = map.get(DcMotor.class, "left_back");
        rightMotorFront = map.get(DcMotor.class, "right_front");
        rightMotorBack = map.get(DcMotor.class, "right_back");

        leftLift = map.get(DcMotor.class, "left_lift");//giveing the motors a name for codeing
        rightLift = map.get(DcMotor.class, "right_lift");

        extendArmMotor = map.get(DcMotor.class, "extend_arm");
        armPivotMotor = map.get(DcMotor.class, "pivot_arm");

        //set encoders to 0 on init
        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightLift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        extendArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Set RunModes for Encoder Usage
        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightLift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        extendArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armPivotMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //Set Direction of each Motors
        // switch REVERSE and FORWARD if controls are opposite
        leftMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotorBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotorFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotorBack.setDirection(DcMotorSimple.Direction.FORWARD);

        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);

        extendArmMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        armPivotMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        //Servos for intake on the map
        topIntake = map.get(CRServo.class, "top_intake");
        bottomIntake = map.get(CRServo.class, "bottom_intake");
    }

    /**
     * Set drive train power for mechanum drive
     * @param frontLeftPower
     * @param backLeftPower
     * @param frontRightPower
     * @param backRightPower
     */
    public void setDriveTrain(
           double frontLeftPower, double backLeftPower,
           double frontRightPower, double backRightPower
    ) {
       leftMotorFront.setPower(frontLeftPower);
       leftMotorBack.setPower(backLeftPower);
       rightMotorFront.setPower(frontRightPower);
       rightMotorBack.setPower(backRightPower);
    }

    /**
     * Set Lift Power for hang
     * @param liftPower
     */
    public void setLift(
            double liftPower
    ){
        leftLift.setPower(liftPower);
        rightLift.setPower(liftPower);
    }

    /**
     * Set intake power
     * @param intakePower
     */
    public void setIntakePosition(
            double intakePower
    ) {
        topIntake.setPower(intakePower);
        bottomIntake.setPower(-intakePower);
    }

    /**
     * Run extention arm
     * @param power
     */
    public void setExtendPower(double power){
        extendArmMotor.setPower(power);
    }

    public double getArmPosition(){
        int currentTicks = extendArmMotor.getCurrentPosition();
        double revolutions = (double) currentTicks / TICKS_PER_REV;
        return revolutions * ARM_GEAR_RATIO * DISTANCE_PER_REV;
    }

    /**
     * Run extension arm based on target position
     * @param targetPosition
     * @param power
     */
    public void autoPivotArm(
            int targetPosition, double power
    ) {
        armPivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        armPivotMotor.setTargetPosition(targetPosition);
        armPivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        armPivotMotor.setPower(power);
    }

    /**
     * Run Pivot Motor
     * @param power
     */
    public void setPivotPower(double power){
        armPivotMotor.setPower(power);
    }

    /**
     * Get posiiton of extention motor
     * @return encoder tick of extention motor
     */
    public double getExtendPos(){
        return extendArmMotor.getCurrentPosition();
    }

    /**
     * Get position of pivot motor
     * @return encoder tick of pivot motor
     */
    public double getPivotArmPos(){
        return armPivotMotor.getCurrentPosition();
    }

    /**
     * Get position of left lift motor
     * @return encoder tick of left lift motor
     */
    public double getLeftLiftPos() { return leftLift.getCurrentPosition();}

    /**
     * Get position of right lift motor
     * @return encoder tick of right lift motor
     */
    public double getRightLiftPos() { return rightLift.getCurrentPosition();}


}