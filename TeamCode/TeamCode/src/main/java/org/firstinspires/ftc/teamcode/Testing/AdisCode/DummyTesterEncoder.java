package org.firstinspires.ftc.teamcode.Testing.AdisCode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Autonomous(name="DummyTesterEncoder", group = "tool")
//@Disabled
public class DummyTesterEncoder extends LinearOpMode
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
        grip = hardwareMap.get(Servo.class, "grip");

        grip.setPosition(0.7);





        waitForStart();


        // telemetry.addData("Desired Tag", DESIRED_TAG_ID);
        // telemetry.update();




        while (opModeIsActive()) {
           // arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lf_drive.setDirection(DcMotorSimple.Direction.REVERSE);
            lb_drive.setDirection(DcMotorSimple.Direction.REVERSE);

          //  arm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lb_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            rb_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            lf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lb_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rf_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rb_drive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


            encoderDriveForward(300);//Moves the robot forward the amount of milimeters specified
            encoderDriveRight(300);//Strafs the robot right the amount of milimeters specified
            encoderDriveLeft(300);//Strafs the robot left the amount of milimeters specified
            encoderDriveBackward(300);//Moves the robot backward the amount of milimeters specified
            encoderDriveBackward(900);

            sleep(30000);

        }

    }
    public void encoderDriveForward(double mm){
        double TotalTicks = mm*COUNTS_PER_MM;
        lf_drive.setTargetPosition((int)TotalTicks);
        lb_drive.setTargetPosition((int)TotalTicks);
        rf_drive.setTargetPosition((int)TotalTicks);
        rb_drive.setTargetPosition((int)TotalTicks);
        lf_drive.setPower(0.5);
        lb_drive.setPower(0.5);
        rf_drive.setPower(0.5);
        rb_drive.setPower(0.5);
        lf_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep((long)(mm*2.5));
        lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }
    public void encoderDriveBackward(double mm){
        double TotalTicks = mm*COUNTS_PER_MM;
        lf_drive.setTargetPosition(-((int)TotalTicks));
        lb_drive.setTargetPosition(-((int)TotalTicks));
        rf_drive.setTargetPosition(-((int)TotalTicks));
        rb_drive.setTargetPosition(-((int)TotalTicks));
        lf_drive.setPower(0.5);
        lb_drive.setPower(0.5);
        rf_drive.setPower(0.5);
        rb_drive.setPower(0.5);
        lf_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep((long)(mm*2.5));
        lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void encoderDriveRight(double mm){
        double TotalTicks = mm*COUNTS_PER_MM;
        lf_drive.setTargetPosition(((int)TotalTicks));
        lb_drive.setTargetPosition(-((int)TotalTicks));
        rf_drive.setTargetPosition(-((int)TotalTicks));
        rb_drive.setTargetPosition(((int)TotalTicks));
        lf_drive.setPower(0.5);
        lb_drive.setPower(0.5);
        rf_drive.setPower(0.5);
        rb_drive.setPower(0.5);

        lf_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep((long)(mm*2.5));
        lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    public void encoderDriveLeft(double mm){
        double TotalTicks = mm*COUNTS_PER_MM;
        lf_drive.setTargetPosition(-((int)TotalTicks));
        lb_drive.setTargetPosition(((int)TotalTicks));
        rf_drive.setTargetPosition(((int)TotalTicks));
        rb_drive.setTargetPosition(-((int)TotalTicks));
        lf_drive.setPower(0.5);
        lb_drive.setPower(0.5);
        rf_drive.setPower(0.5);
        rb_drive.setPower(0.5);

        lf_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lb_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rf_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rb_drive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep((long)(mm*2.5));
        lf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lb_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rf_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rb_drive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }


}
