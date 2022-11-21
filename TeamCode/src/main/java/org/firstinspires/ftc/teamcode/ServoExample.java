package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.CRServo;



@TeleOp(name="ServoExample", group="Linear Opmode")

public class ServoExample extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor liftMotor = null;
    private CRServo servoGrabber = null;
    private CRServo servoLevel = null;

    static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int    CYCLE_MS    =   50;     // period of each cycle
    static final double MAX_POS     =  1.0;     // Maximum rotational position
    static final double MIN_POS     =  0.0;

    int direction = 0;
    double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean liftOn = false;


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
        liftMotor = hardwareMap.get(DcMotor.class, "motor_lift");
        servoGrabber = hardwareMap.get(CRServo.class, "servo_grabber");
        servoLevel = hardwareMap.get(CRServo.class, "servo_level");

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
        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double left  = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double right =  - gamepad1.right_stick_y;

            double lift = -gamepad2.right_stick_y;
            boolean servoOpen = gamepad2.dpad_up;
            boolean servoClose = gamepad2.dpad_down;


            //Code for arm (gamepad2)
            if(servoOpen){
                direction = -1;
                position = position +.05;
                if(position > 1.0)position=1.0;

                telemetry.addData("Forward", "servo: " + position);

                servoGrabber.setPower(0.25*direction);
                waitTime(.5);
                servoGrabber.setPower(0);
            }

            if(servoClose){
                direction = 1;
                position= position -0.05;
                if(position < 0.0)position=0.0;

                telemetry.addData("back", "servo: " + position);

                servoGrabber.setPower(0.25*direction);
                waitTime(.5);
                servoGrabber.setPower(0);
            }

            if(lift>0){
                liftMotor.setPower(.05);
            }else if(lift<0){
                liftMotor.setPower(-.05);
            }

            //Code for drivetrain (gamepad1)
            /*if(left<.05)left = 0;
            if(right<.05)right= 0;
            */

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

            // More test code:
            //
            // This is not used for actual robotics thing, its merely for testing different call functions
            // Also completely screws over the code because of the while loop, therefore
            // DO NOT HAVE THIS WHILE LOOP UNCOMMENTED WHEN ACTUALLY USING THE ROBOT!!!!
            // DO NOT HAVE THIS WHILE LOOP UNCOMMENTED WHEN ACTUALLY USING THE ROBOT!!!!
            // DO NOT HAVE THIS WHILE LOOP UNCOMMENTED WHEN ACTUALLY USING THE ROBOT!!!!
            // DO NOT HAVE THIS WHILE LOOP UNCOMMENTED WHEN ACTUALLY USING THE ROBOT!!!!
            // spam for extra effect and hopefully notice it better

            /*
            while(true){
                telemetry.addData("Current Position: ",leftFrontDrive.getCurrentPosition());
                leftFrontDrive.setTargetPosition(leftFrontDrive.getCurrentPosition() + 360);
            }
            */


            // Send calculated power to wheels
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.update();
        }
    }
}