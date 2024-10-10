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

    private HardwareMap hwMap = null;


    //Statistics for measurements



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

        hwMap = map;

        //Connecting declared motors to classes of DcMotors and respected names
        leftMotorFront = hwMap.get(DcMotor.class, "left_front");
        leftMotorBack = hwMap.get(DcMotor.class, "left_back");
        rightMotorFront = hwMap.get(DcMotor.class, "right_front");
        rightMotorBack = hwMap.get(DcMotor.class, "right_back");
//        leftLift = hwMap.get(DcMotor.class, "left_lift");//giveing the motors a name for codeing
//        rightLift = hwMap.get(DcMotor.class, "right_lift");

        leftMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightMotorBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //Set RunModes for Encoder Usage
        /*
        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         */

        //Set Direction of each Motors
        // switch REVERSE and FORWARD if controls are opposite
        // This is set for Mechanum drive
        leftMotorFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftMotorBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotorFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightMotorBack.setDirection(DcMotorSimple.Direction.FORWARD);
//        leftLift.setDirection(DcMotorSimple.Direction.FORWARD);
//        rightLift.setDirection(DcMotorSimple.Direction.REVERSE);//this is because the motors are probalby faceing each other

        //Servos for intake on the map
//        topIntake = hwMap.get(CRServo.class, "top_intake");
//        bottomIntake = hwMap.get(CRServo.class, "bottom_intake");
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

    public void setLift(
            double liftPower
    ){
        leftLift.setPower(liftPower);
        rightLift.setPower(liftPower);
    }

    public void setIntakePosition(
            double intakePower
    ) {
        topIntake.setPower(intakePower);
        bottomIntake.setPower(-intakePower);
    }


    //Drive Encoder Stats
    static final double COUNTS_PER_MOTOR_REV = 537.7;    // GoBilda 5203 motor encoder res
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // 1:1
    static final double WHEEL_DIAMETER_INCHES = 4.09449;     // For figuring circumference
    static final double CIRCUMFERENCE = (WHEEL_DIAMETER_INCHES * Math.PI);
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) / CIRCUMFERENCE;
    static final double DISTANCE_PER_ENCODER =  CIRCUMFERENCE/COUNTS_PER_MOTOR_REV;
    static final double COUNTS_PER_DEGREE = COUNTS_PER_MOTOR_REV/360;
    static final double INCHES_PER_DEGREE = DISTANCE_PER_ENCODER * COUNTS_PER_DEGREE;


    /**
     * Drive using encoders for auto
     * @param speed
     * @param distance
     */
    public void encoderDrive(double speed, double distance) {
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;

        //calculate distance to encoder ticks
        newfrontLeftTarget = leftMotorFront.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
        newfrontRightTarget = rightMotorFront.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
        newbackLeftTarget = leftMotorBack.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
        newbackRightTarget = rightMotorBack.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);

        // set target position to hit for distance given
        leftMotorFront.setTargetPosition(newfrontLeftTarget);
        rightMotorFront.setTargetPosition(newfrontRightTarget);
        leftMotorBack.setTargetPosition(newbackLeftTarget);
        rightMotorBack.setTargetPosition(newbackRightTarget);

        // Turn On RUN_TO_POSITION
        leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // start motion
        leftMotorFront.setPower(Math.abs(speed));
        rightMotorFront.setPower(Math.abs(speed));
        leftMotorBack.setPower(Math.abs(speed));
        rightMotorBack.setPower(Math.abs(speed));

        // Telemetry
        while (opMode.opModeIsActive() &&
                (leftMotorFront.isBusy() && rightMotorFront.isBusy() && leftMotorBack.isBusy() && rightMotorBack.isBusy())) {
            opMode.telemetry.addData("frontLeftEncoder", leftMotorFront.getCurrentPosition());
            opMode.telemetry.addData("frontRightEncoder", rightMotorFront.getCurrentPosition());
            opMode.telemetry.addData("backLeftEncoder", leftMotorBack.getCurrentPosition());
            opMode.telemetry.addData("backRightEncoder", rightMotorBack.getCurrentPosition());
            opMode.telemetry.update();
        }


        // Stop all motion;
        leftMotorFront.setPower(0);
        rightMotorFront.setPower(0);
        leftMotorBack.setPower(0);
        rightMotorBack.setPower(0);

        // Turn off RUN_TO_POSITION
        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set encoders to 0 when function is completed
        // we do this to ensure the next set of functions are based at 0 instead of taking
        // the encoder value the function finishes at
        resetEncoder();
    }

    /**
     * Turn using encoders for auto (pivot)
     * @param speed
     * @param degrees
     */
    public void encoderTurn(double speed, double degrees) {

        //calculate target count based on degrees to turn
        double turnCircumference = Math.PI * (degrees * 4 / 360) * (WHEEL_DIAMETER_INCHES * 2);
        int targetCounts = (int) (turnCircumference / DISTANCE_PER_ENCODER);

        // set target position of encoders based on degree to turn
        leftMotorFront.setTargetPosition(-targetCounts);
        rightMotorFront.setTargetPosition(targetCounts);
        leftMotorBack.setTargetPosition(-targetCounts);
        rightMotorBack.setTargetPosition(targetCounts);

        // Turn On RUN_TO_POSITION
        leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        leftMotorFront.setPower(Math.abs(speed));
        rightMotorFront.setPower(Math.abs(speed));
        leftMotorBack.setPower(Math.abs(speed));
        rightMotorBack.setPower(Math.abs(speed));

        while (opMode.opModeIsActive() &&
                (leftMotorFront.isBusy() && leftMotorBack.isBusy() && rightMotorFront.isBusy() && rightMotorBack.isBusy())) {
            opMode.telemetry.addData("frontLeftEncoder", leftMotorFront.getCurrentPosition());
            opMode.telemetry.addData("frontRightEncoder", rightMotorFront.getCurrentPosition());
            opMode.telemetry.addData("backLeftEncoder", leftMotorBack.getCurrentPosition());
            opMode.telemetry.addData("backRightEncoder", rightMotorBack.getCurrentPosition());
            opMode.telemetry.update();
        }

        // Stop all motion;
        leftMotorFront.setPower(0);
        rightMotorFront.setPower(0);
        leftMotorBack.setPower(0);
        rightMotorBack.setPower(0);

        // Turn off RUN_TO_POSITION
        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set encoders to 0 when function is completed
        // we do this to ensure the next set of functions are based at 0 instead of taking
        // the encoder value the function finishes at
        resetEncoder();
    }

    /**
     * Strafe using encoders for auto
     * @param speed
     * @param distance
     */
    public void encoderStrafe(double speed, double distance) {
        int newfrontLeftTarget;
        int newfrontRightTarget;
        int newbackLeftTarget;
        int newbackRightTarget;


        // Determine new target position, and pass to motor controller
        newfrontLeftTarget = leftMotorFront.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);
        newfrontRightTarget = rightMotorFront.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
        newbackLeftTarget = leftMotorBack.getCurrentPosition() + (int) (-distance * COUNTS_PER_INCH);
        newbackRightTarget = rightMotorBack.getCurrentPosition() + (int) (distance * COUNTS_PER_INCH);

        //Set target position for motors
        leftMotorFront.setTargetPosition(newfrontLeftTarget);
        rightMotorFront.setTargetPosition(newfrontRightTarget);
        leftMotorBack.setTargetPosition(newbackLeftTarget);
        rightMotorBack.setTargetPosition(newbackRightTarget);

        // Turn On RUN_TO_POSITION
        leftMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // reset the timeout time and start motion.
        leftMotorFront.setPower(Math.abs(speed));
        rightMotorFront.setPower(-Math.abs(speed));
        leftMotorBack.setPower(-Math.abs(speed));
        rightMotorBack.setPower(Math.abs(speed));

        //Telemetry
        while (opMode.opModeIsActive() &&
                (leftMotorFront.isBusy() && leftMotorBack.isBusy() && rightMotorFront.isBusy() && rightMotorBack.isBusy())) {
            opMode.telemetry.addData("frontLeftEncoder", leftMotorFront.getCurrentPosition());
            opMode.telemetry.addData("frontRightEncoder", rightMotorFront.getCurrentPosition());
            opMode.telemetry.addData("backLeftEncoder", leftMotorBack.getCurrentPosition());
            opMode.telemetry.addData("backRightEncoder", rightMotorBack.getCurrentPosition());
            opMode.telemetry.update();
            }

        // Stop all motion;
        leftMotorFront.setPower(0);
        rightMotorFront.setPower(0);
        leftMotorBack.setPower(0);
        rightMotorBack.setPower(0);

        // Turn off RUN_TO_POSITION
        leftMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // set encoders to 0 when function is completed
        // we do this to ensure the next set of functions are based at 0 instead of taking
        // the encoder value the function finishes at
        resetEncoder();
    }

    private void resetEncoder(){
        leftMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void setDrivePower(double left, double right){
        leftMotorFront.setPower(left);
        leftMotorBack.setPower(left);
        rightMotorFront.setPower(right);
        rightMotorBack.setPower(right);

        leftMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightMotorFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}
