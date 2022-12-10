package org.firstinspires.ftc.teamcode.Helper;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;


public class NewRobot {
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor verticalSlider;
    public DcMotor horizontalSlider;
    public Servo claw;

    //IMU
    public static BNO055IMU imu;
    public Orientation angles;
    public Acceleration gravity;
    static final double MAX_POS     =  1.0;
    static final double MIN_POS     =  0.0;

    //How many times the encoder counts a tick per revolution of the motor.
    static final double COUNTS_PER_MOTOR_REV    = 538;      // eg: GoBuilda 5203 Planetery

    //Gear ratio of the motor to the wheel. 1:1 would mean that 1 turn of the motor is one turn of the wheel, 2:1 would mean two turns of the motor is one turn of the wheel, and so on.
    static final double DRIVE_GEAR_REDUCTION    = 1;        // This is < 1.0 if geared UP

    //Diameter of the wheel in CM
    static final double WHEEL_DIAMETER_CM   = 10;     // For figuring circumference

    //How many times the encoder counts a tick per CM moved. (Ticks per rev * Gear ration) / perimeter
    static final double COUNTS_PER_CM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_CM * 3.1415);



    /* local OpMode members. */

    //Init hardware map
    HardwareMap hwMap = null;
    HardwareMap imuHwMap = null;

    public ElapsedTime period = new ElapsedTime();
    //tells you how long the robot has run for
    public ElapsedTime test_run_time = new ElapsedTime();
    //this is how you create an instance in a java class ^


    //
    public void Robot() {

    }

    public enum MoveStep {
        yaxix, xaxis, turn, stop
    }

    public MoveStep moveStep;

    //private static LinearOpmode opModeObj;

    public void init(HardwareMap ahwMap) throws InterruptedException {

        hwMap = ahwMap;
        //Init motors and servos
        claw = hwMap.get(Servo.class, "claw");
        verticalSlider = hwMap.get(DcMotor.class, "verticalSlider");
        horizontalSlider = hwMap.get(DcMotor.class, "horizontalSlider");


        //Setting the run mode
        claw.setDirection(Servo.Direction.FORWARD);
        verticalSlider.setDirection(DcMotorSimple.Direction.FORWARD);
        horizontalSlider.setDirection(DcMotorSimple.Direction.FORWARD);
    }
}