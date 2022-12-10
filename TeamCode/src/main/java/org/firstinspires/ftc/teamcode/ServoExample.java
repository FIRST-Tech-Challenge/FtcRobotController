package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;



@TeleOp(name="ServoExample", group="Linear Opmode")

public class ServoExample extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor liftMotor = null;
    private Servo servoGrabber1 = null;
    private Servo servoGrabber2 = null;

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  180;     // Maximum rotational position
    static final double MAX_POS2    =    0;
    static final double MIN_POS     =  0.0;

    int direction = 0;
    double  position = 0;

    int strafeDirection = 0; //-1 for left, 1 for right

    public void waitTime(double time){
        runtime.reset();
        while(runtime.seconds()<time){
        }
    }

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");

        servoGrabber1 = hardwareMap.get(Servo.class, "servo_grabber_one");
        servoGrabber2 = hardwareMap.get(Servo.class,"servo_grabber_two");
        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);

        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // If making a new Auto or TeleOp file, make sure to include waitForStart so it can pass
        // inspection and you don't get dq-ed from tourneys (mainly for Auto)
        waitForStart();
        runtime.reset();

        // Run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            // Look through old files to try to find POV Mode. Currently using tank drive code
            double left  = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double right =  - gamepad1.right_stick_y;
            boolean strafeLeft = gamepad1.left_bumper;
            boolean strafeRight = gamepad1.right_bumper;

            //Switch this to gamepad1 to test if it works in the first place
            boolean servoOpen = gamepad2.dpad_up;
            boolean servoClose = gamepad2.dpad_down;

            double lift = -gamepad2.right_stick_y;

            // Code for lift and grabber (gamepad2
            if(servoOpen){
                direction = -1;
                if(position<MAX_POS) {
                    position = position + 0.05;
                }
                if(position > 1.0){
                    position=1.0;
                }
                telemetry.addData("Forward", "servo: " + position);
                servoGrabber1.setPosition(position);
            }

            if(servoClose){
                direction = 1;
                position= position -0.05;
                if(position < 0.0){
                    position=0.0;
                }
                telemetry.addData("back", "servo: " + position);
                servoGrabber1.setPosition(position);
            }

            if(lift > .05){
                liftMotor.setPower(1);
            }else{
                liftMotor.setPower(0);
            }
            if(lift < -.05){
                liftMotor.setPower(-.3);
            }else{
                liftMotor.setPower(0);
            }

            //Code for drivetrain (gamepad1)
            if(left < .05 && left> -.05){
                left = 0;
            }
            if(right < .05 && right > -.05){
                right= 0;
            }

            //makes the stuff non-linear
            if(left>0){
                left *=left;
            }else{
                left = left*left*-1;
            }

            if(right>0){
                right *=right;
            }else{
                right = right*right*-1;
            }

            if(strafeLeft || strafeRight){
                if(strafeLeft){
                    strafeDirection = -1;
                }else{
                    strafeDirection = 1;
                }
            }else{
                strafeDirection = 0;
            }
            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = left;
            double rightFrontPower = right;
            double leftBackPower   = left;
            double rightBackPower  = right;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            leftFrontPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            leftBackPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            rightFrontPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            rightBackPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */

            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower+strafeDirection);
            rightFrontDrive.setPower(rightFrontPower-strafeDirection);
            leftBackDrive.setPower(leftBackPower-strafeDirection);
            rightBackDrive.setPower(rightBackPower+strafeDirection);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
}