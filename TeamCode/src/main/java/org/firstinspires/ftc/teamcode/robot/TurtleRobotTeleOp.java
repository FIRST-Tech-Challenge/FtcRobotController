package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TurtleRobotTeleOp {
    /* Public OpMode members. */
    public DcMotor rightfrontmotor = null;
    public DcMotor rightbackmotor = null;
    public DcMotor leftfrontmotor = null;
    public DcMotor leftbackmotor = null;
    public Servo ArmServo = null;
    public DcMotor leftslidemotor = null;
    public DcMotor rightslidemotor = null;
    public ElapsedTime runtime = new ElapsedTime();
    public LinearOpMode myOpMode = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    public ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public TurtleRobotTeleOp(LinearOpMode opmode) { myOpMode = opmode;
    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {

        // Save reference to Hardware map
        hwMap = ahwMap;
        // Define and Initialize Motors
        leftfrontmotor = hwMap.get(DcMotor.class, "leftfrontmotor");
        leftbackmotor = hwMap.get(DcMotor.class, "leftbackmotor");
        rightfrontmotor = hwMap.get(DcMotor.class, "rightfrontmotor");
        rightbackmotor = hwMap.get(DcMotor.class, "rightbackmotor");
        ArmServo = hwMap.get(Servo.class, "ArmServo");
        leftslidemotor = hwMap.get(DcMotor.class, "leftEncoder");
        rightslidemotor = hwMap.get(DcMotor.class, "rightslidemotor");

        leftfrontmotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftbackmotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightfrontmotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightbackmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftslidemotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightslidemotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all motors to zero power
        leftfrontmotor.setPower(0);
        leftbackmotor.setPower(0);
        rightfrontmotor.setPower(0);
        rightbackmotor.setPower(0);
        ArmServo.setPosition(0);
        leftslidemotor.setPower(0);
        rightslidemotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
//        leftbackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        leftfrontmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightfrontmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        rightbackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

}