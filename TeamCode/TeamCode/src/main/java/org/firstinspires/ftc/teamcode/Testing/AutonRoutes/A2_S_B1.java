package org.firstinspires.ftc.teamcode.Testing.AutonRoutes;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Testing.Helper_test.AdisEncoderDrive;

//This program was made by Adi. It is meant to be where we can acess documentation about the encoder drive functions
@Autonomous(name="A2_S_B1", group = "tool")
@Disabled
public class A2_S_B1 extends LinearOpMode
{
    // Adjust these numbers to suit your robot.

    static final double     COUNTS_PER_MOTOR_REV    = 537.7 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // No External Gearing.
    static final double     WHEEL_DIAMETER_MM   = 96.0 ;     // For figuring circumference
    static final double     COUNTS_PER_MM         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_MM * 3.1415);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;










    private DcMotor lf_drive   = null;  //  Used to control the left front drive wheel
    private DcMotor rf_drive  = null;  //  Used to control the right front drive wheel
    private DcMotor lb_drive    = null;  //  Used to control the left back drive wheel
    private DcMotor rb_drive   = null;//  Used to control the right back drive wheel


    //private DcMotor arm = null; // Used to control arm
    Servo grip; // Used to control gripper


    @Override
    public void runOpMode()
    {
        boolean targetFound     = false;    // Set to true when an AprilTag target is detected
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double  turn            = 0;        // Desired turning power/speed (-1 to +1)

        // Initialize the Apriltag Detection process


        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must match the names assigned during the robot configuration.
        // step (using the FTC Robot Controller app on the phone).
        lf_drive = hardwareMap.get(DcMotor.class, "lf_drive");
        rf_drive = hardwareMap.get(DcMotor.class, "rf_drive");
        lb_drive = hardwareMap.get(DcMotor.class, "lb_drive");
        rb_drive = hardwareMap.get(DcMotor.class, "rb_drive");
        //arm = hardwareMap.get(DcMotor.class, "arm");
       // grip = hardwareMap.get(Servo.class, "grip");

       // grip.setPosition(0.7);





        waitForStart();



        // telemetry.addData("Desired Tag", DESIRED_TAG_ID);
        // telemetry.update();


        AdisEncoderDrive adisEncoderDrive = new AdisEncoderDrive();


        while (opModeIsActive()) {
            //place pixel and return back to the block to the left of the wing(human player station) tape

            adisEncoderDrive.encoderDriveForwardInches(2.5);
            adisEncoderDrive.encoderDriveLeftBlocks(3);
            adisEncoderDrive.encoderDriveForwardBlocks(1.2);
            adisEncoderDrive.TurnLeft(90);
            //detect apriltags
            //place pixel on apriltags
            //soon create an if statment wheer it strafes left according to the apriltag Id
            adisEncoderDrive.encoderDriveLeftBlocks(1);
            adisEncoderDrive.encoderDriveForwardBlocks(0.5);
            sleep(3000);





        }

    }


}
