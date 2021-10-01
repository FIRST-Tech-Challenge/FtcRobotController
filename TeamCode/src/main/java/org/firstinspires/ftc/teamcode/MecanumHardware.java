package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This is NOT an opmode.
 *
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a K9 robot.
 *
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 *
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Servo channel:  Servo to raise/lower arm: "arm"
 * Servo channel:  Servo to open/close claw: "claw"
 *
 * Note: the configuration of the servos is such that:
 *   As the arm servo approaches 0, the arm position moves up (away from the floor).
 *   As the claw servo approaches 0, the claw opens up (drops the game element).
 */
public class MecanumHardware
{
    /* Public OpMode members. */
    public DcMotor  leftFront  = null;
    public DcMotor  rightFront  = null;
    public DcMotor  rightBack  = null;
    public DcMotor  leftBack  = null;
    public DcMotor launcher1 = null;
    public DcMotor launcher2 = null;
    public DcMotor intake = null;
    public DcMotor arm = null;

    // Extra motors added to the 2nd Expansion Hub
    /*public DcMotor  ActuatorMotor = null;
    public DcMotor  ElbowMotor = null;
    public DcMotor  HeightMotor = null;*/

    // Servos
    public Servo    triggerServo         = null;
    public CRServo conveyServo = null;
    public Servo grabberServo = null;


    // Variable for servo position
    public final static double Servo_Close = 1;
    public final static double Servo_Open = 0;

    /*
    public final static double ARM_MIN_RANGE  = 0.20;
    public final static double ARM_MAX_RANGE  = 0.90;
    public final static double CLAW_MIN_RANGE  = 0.20;
    public final static double CLAW_MAX_RANGE  = 0.7;
    */

    /* Local OpMode members. */
    HardwareMap hwMap  = null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public MecanumHardware() {
    }


    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // save reference to HW Map
        hwMap = ahwMap;

        // Define and Initialize Motors
        leftFront = hwMap.dcMotor.get("LeftFront");
        leftBack = hwMap.dcMotor.get("LeftBack");
        rightFront = hwMap.dcMotor.get("RightFront");
        rightBack = hwMap.dcMotor.get("RightBack");
        launcher1 = hwMap.dcMotor.get("Launcher1");
        launcher2 = hwMap.dcMotor.get("Launcher2");
        intake = hwMap.dcMotor.get("Intake");
        arm = hwMap.dcMotor.get("Arm");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        arm.setDirection(DcMotor.Direction.REVERSE);
        launcher1.setDirection(DcMotor.Direction.REVERSE);






        // Set all motors to zero power
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
        launcher1.setPower(0);
        launcher2.setPower(0);
        intake.setPower(0);
        arm.setPower(0);



        // Color and Distance Sensor
        //FrontColorSensor = hwMap.colorSensor.get("Color0");
        //FrontDistanceSensor = hwMap.opticalDistanceSensor.get("Color0");


        /*
        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        */

       // Define and initialize ALL installed servos.
        triggerServo = hwMap.get(Servo.class, "TriggerServo");
        triggerServo.setPosition(Servo_Close);
        conveyServo = hwMap.get(CRServo.class, "ConveyServo");
        conveyServo.setDirection(CRServo.Direction.REVERSE);
        conveyServo.setPower(0);
        grabberServo = hwMap.get(Servo.class, "GrabberServo");
        grabberServo.setPosition(Servo_Close);

    }
}


