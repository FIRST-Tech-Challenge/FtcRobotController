package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

public class Hardware
{
    //drive motor declaration
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    //servo declaration
    public Servo depositServoOne;
    public Servo depositServoTwo;

    //helper class variables
    public double x = 0, y = 0, theta = 0;
    public static LinearOpMode currentOpMode;

    public DcMotorEx leftOdom, rightOdom, centerOdom;

    // Real world distance traveled by the wheels
    public double leftOdomTraveled, rightOdomTraveled, centerOdomTraveled;

    // Odometry encoder positions
    public int leftEncoderPos, centerEncoderPos, rightEncoderPos;

    //used in our odo but not RoadRunner classes
    public static final double ODOM_TICKS_PER_IN = 1831.471685;
    public static double trackwidth = 10.976;

    public Hardware(HardwareMap hardwareMap)
    {
        //Deposit servo config
        depositServoOne = hardwareMap.get(Servo.class, "Left Deposit");
        depositServoTwo = hardwareMap.get(Servo.class, "Right Deposit");

        //drive motor initialization
        frontLeft = hardwareMap.get(DcMotorEx.class, "Front Left");
        frontRight = hardwareMap.get(DcMotorEx.class, "Front Right");
        backLeft = hardwareMap.get(DcMotorEx.class, "Back Left");
        backRight = hardwareMap.get(DcMotorEx.class, "Back Right");

        //odom
        leftOdom = hardwareMap.get(DcMotorEx.class, "Front Left");
        rightOdom = hardwareMap.get(DcMotorEx.class, "Front Right");
        centerOdom = hardwareMap.get(DcMotorEx.class, "Back Right");
    }

    //robot-oriented drive method
    public void robotODrive(double forward, double sideways, double rotation)
    {
        //adds all the inputs together to get the number to scale it by
        double scale = Math.abs(rotation) + Math.abs(forward) + Math.abs(sideways);

        //scales the inputs when needed
        if (scale > 1)
        {
            forward /= scale;
            rotation /= scale;
            sideways /= scale;
        }
        //setting the motor powers to move
        frontLeft.setPower(forward - rotation - sideways);
        backLeft.setPower(forward - rotation + sideways);
        frontRight.setPower(forward + rotation + sideways);
        backRight.setPower(forward + rotation - sideways);
    }
}
