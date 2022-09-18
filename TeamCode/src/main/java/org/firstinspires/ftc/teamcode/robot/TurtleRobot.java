package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class TurtleRobot {
    /* Public OpMode members. */
    public DcMotor rightfrontmotor = null;
    public DcMotor rightbackmotor = null;
    public DcMotor leftfrontmotor = null;
    public DcMotor leftbackmotor = null;
    public ElapsedTime runtime = new ElapsedTime();
    public LinearOpMode myOpMode = null;

    /* local OpMode members. */
    HardwareMap hwMap = null;
    public ElapsedTime period = new ElapsedTime();

    /* Constructor */
    public TurtleRobot (LinearOpMode opmode) { myOpMode = opmode;
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

        leftfrontmotor.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        leftbackmotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        rightfrontmotor.setDirection(DcMotorSimple.Direction.FORWARD);
        rightbackmotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Set all motors to zero power
        leftfrontmotor.setPower(0);
        leftbackmotor.setPower(0);
        rightfrontmotor.setPower(0);
        rightbackmotor.setPower(0);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftbackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftbackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightfrontmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightbackmotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }
}