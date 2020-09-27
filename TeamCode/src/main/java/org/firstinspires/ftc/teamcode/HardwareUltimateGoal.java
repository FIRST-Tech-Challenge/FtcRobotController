package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a RoverRuckus.
 *
 */
public class HardwareUltimateGoal  {

    /* Public OpMode members. */
    // drivetrain motors
    public DcMotor frontLeft = null;
    public DcMotor frontRight = null;
    public DcMotor backLeft = null;
    public DcMotor backRight = null;

    // input, output motors
    public DcMotor intake = null;
    public DcMotor outtake = null;

    private boolean test;


    public ModernRoboticsI2cGyro realgyro;
    public ModernRoboticsI2cGyro realgyro2;


    public static final String TeleOpRunMode = "no encoders";



    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  =  new ElapsedTime();

    /* Constructor */
    public HardwareUltimateGoal(boolean test){
        this.test = test;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;

        // Define and Initialize Motors

        //Drive Train motors
        frontLeft = hwMap.get(DcMotor.class, "front_left");
        frontRight = hwMap.get(DcMotor.class, "front_right");
        backRight = hwMap.get(DcMotor.class, "back_right");
        backLeft = hwMap.get(DcMotor.class, "back_left");
        intake = hwMap.get(DcMotor.class, "succ");
        outtake = hwMap.get(DcMotor.class, "spit");




        if (!test) {
            //Define and Initialize Sensors
            realgyro = hwMap.get(ModernRoboticsI2cGyro.class, "gyro");
            realgyro.calibrate();


//            realgyro2 = hwMap.get(ModernRoboticsI2cGyro.class, "gyro2");
//            realgyro2.calibrate();






        }


        frontLeft.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        frontRight.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        backRight.setDirection(DcMotor.Direction.FORWARD);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        outtake.setDirection(DcMotor.Direction.FORWARD);

        if (!test) {


        }



        // Set all motors to zero power
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);
        intake.setPower(0);
        outtake.setPower(0);



        if (!test) {
            realgyro.resetZAxisIntegrator();



        }

    }
    public void setMode(String mode){
        if (mode.equals(TeleOpRunMode)){

            frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if (!test) {


            }
        }
        else {

            // Reset encoder
            frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            outtake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            if (!test) {


            }


            // set to run using encoder
            frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (!test) {


            }


        }
    }
}
