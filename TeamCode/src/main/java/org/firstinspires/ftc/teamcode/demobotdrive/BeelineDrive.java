package org.firstinspires.ftc.teamcode.demobotdrive;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name="Beeline", group="Linear Opmode")
public class BeelineDrive extends LinearOpMode {


    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;

    public float speedmulti = .5f;
    public float Yout = 0;


    //private Servo fireServo = null;
    //private CRServo barrel = null;
    TouchSensor limit;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "LeftFrontDrive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "LeftBackDrive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "RightFrontDrive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "RightBackDrive");
        //fireServo = hardwareMap.get(Servo.class, "fire_servo");
        //barrel = hardwareMap.get(CRServo.class, "barrel" );

        // Most robots need the motors on one side to be reversed to drive forward.
        // When you first test your robot, push the left joystick forward
        // and flip the direction ( FORWARD <-> REVERSE ) of any wheel that runs backwards
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        //fireServo.setDirection(Servo.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");

        /*if (limit.isPressed()) {
            leftFrontDrive.setPower(0);
        } else { // Otherwise, run the motor
            leftFrontDrive.setPower(1);
        }*/


        telemetry.update();

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if(gamepad1.y && speedmulti == 1){
                speedmulti = .5f;
            } else if(gamepad1.y && speedmulti == .5f){
                speedmulti = 1f;
            }

            telemetry.addData("motor Power:", leftBackDrive.getPower());


            double max;


            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            //double vertical   = -gamepad1.right_stick_x;  // Note: pushing stick forward gives negative value
            double vertical     =  gamepad1.left_stick_y;
            double turn     =  gamepad1.left_stick_x;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = vertical  - turn * speedmulti;
            double rightFrontPower = vertical  + turn * speedmulti;
            double leftBackPower   = vertical  - turn * speedmulti;
            double rightBackPower  = vertical  + turn * speedmulti;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));

            
            /* verticle =y
               horizontal = x
               turn = r*/

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
            }
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            //telemetry.addData("limitSwitchState", limitSwitchState);


            //telemetry.addData("CR positon",fireServo.getPosition());
            //telemetry.addData("CR class",fireServo.getClass());
            telemetry.update();
        }
    }}


