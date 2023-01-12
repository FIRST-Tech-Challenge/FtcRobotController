package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.TouchSensor;


//import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

import com.qualcomm.robotcore.util.Range;


/**
 * This file works in conjunction with the External Hardware Class sample called: ConceptExternalHardwareClass.java
 * Please read the explanations in that Sample about how to use this class definition.
 *
 * This file defines a Java Class that performs all the setup and configuration for a sample robot's hardware (motors and sensors).
 * It assumes three motors (left_drive, right_drive and arm) and two servos (left_hand and right_hand)
 *
 * This one file/class can be used by ALL of your OpModes without having to cut & paste the code each time.
 *
 * Where possible, the actual hardware objects are "abstracted" (or hidden) so the OpMode code just makes calls into the class,
 * rather than accessing the internal hardware directly. This is why the objects are declared "private".
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with *exactly the same name*.
 *
 * Or.. In OnBot Java, add a new file named RobotHardware.java, drawing from this Sample; select Not an OpMode.
 * Also add a new OpMode, drawing from the Sample ConceptExternalHardwareClass.java; select TeleOp.
 *
 */

public class Hardware358 {

    /* Declare OpMode members. */
    //private LinearOpMode myOpMode = null;   // gain access to methods in the calling OpMode.

    // Define Motor and Servo objects  (Make them private so they can't be accessed externally)
    public DcMotor lf = null; // left front
    public DcMotor rf = null; // right front
    public DcMotor rb = null; // right back
    public DcMotor lb = null; // left back
   // public DcMotor m = null; //middle
    public DcMotor lift = null;
    //public Servo rightServo = null;
    //public Servo leftServo = null;
    public Servo clawServo = null;
    //CRSERvo
    public CRServo slideServo = null;//for the slideing mechanism
    TouchSensor touch;

//    private DcMotor leftDrive   = null;
//    private DcMotor rightDrive  = null;
//    private DcMotor armMotor = null;
//    private Servo   leftHand = null;
//    private Servo   rightHand = null;

    // Define Drive constants.  Make them public so they CAN be used by the calling OpMode
//    public static final double MID_SERVO       =  0.5 ;
//    public static final double HAND_SPEED      =  0.02 ;  // sets rate to move servo
//    public static final double ARM_UP_POWER    =  0.45 ;
//    public static final double ARM_DOWN_POWER  = -0.45 ;

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public Hardware358() {


    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
   */
    public void init(HardwareMap HwMap) {
        // Define and Initialize Motors (note: need to use reference to actual OpMode).
//        leftDrive  = myOpMode.hardwareMap.get(DcMotor.class, "left_drive");
//        rightDrive = myOpMode.hardwareMap.get(DcMotor.class, "right_drive");
//        armMotor   = myOpMode.hardwareMap.get(DcMotor.class, "arm");
        touch= HwMap.get(TouchSensor.class,"Touch");
        lf = HwMap.get(DcMotor.class, "lf");
        rf = HwMap.get(DcMotor.class, "rf");
        rb = HwMap.get(DcMotor.class, "rb");
        lb = HwMap.get(DcMotor.class, "lb");
        //m = HwMap.get(DcMotor.class, "mid");
        lift = HwMap.get(DcMotor.class, "lift");
        //leftServo = HwMap.get(Servo.class,"leftServo");
        //rightServo = HwMap.get(Servo.class,"rightServo");
        clawServo = HwMap.get(Servo.class,"clawServo");

        //CRSERVO
        slideServo = HwMap.get(CRServo.class, "crservo");

        lf.setDirection(DcMotor.Direction.REVERSE);
        rf.setDirection(DcMotor.Direction.FORWARD);
        lb.setDirection(DcMotor.Direction.REVERSE);
        rb.setDirection(DcMotor.Direction.FORWARD);
        //m.setDirection(DcMotor.Direction.FORWARD);
        lift.setDirection(DcMotor.Direction.FORWARD);

        lf.setPower(0);
        rf.setPower(0);
        lb.setPower(0);
        rb.setPower(0);
        //m.setPower(0);
        lift.setPower(0);
        slideServo.setPower(0);


        //leftServo.setPosition(0.6);
        //rightServo.setPosition(0.34);
        clawServo.setPosition(0.0);

        lf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rf.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rb.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


//        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
//        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
//
//
    }
}

