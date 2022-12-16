// this code is according to https://gm0.org/en/latest/docs/software/tutorials/mecanum-drive.html
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="FakeSwerveV2", group="Linear Opmode")

public class FakeSwerveV2 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFrontDrive = null;
    private DcMotorEx leftBackDrive = null;
    private DcMotorEx rightFrontDrive = null;
    private DcMotorEx rightBackDrive = null;

    private DcMotor liftMotor = null;
    private Servo servoGrabber1 = null;
    private Servo servoGrabber2 = null;

    static final double MAX_POS     =    .5;
    static final double MAX_POS2    =    .50;
    static final double MIN_POS     =     1;
    static final double MIN_POS2    =     0;

    double direction = 0;
    double position = 1;
    double position2 = 0;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotorEx.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotorEx.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "right_back_drive");

        liftMotor  = hardwareMap.get(DcMotor.class, "lift_motor");
        servoGrabber1 = hardwareMap.get(Servo.class, "servo_grabber_one");
        servoGrabber2 = hardwareMap.get(Servo.class, "servo_grabber_two");

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
        leftFrontDrive.setDirection(DcMotorEx.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotorEx.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotorEx.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotorEx.Direction.REVERSE);

        liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        servoGrabber1.setPosition(position);
        servoGrabber2.setPosition(position2);

        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double y  = -gamepad1.left_stick_y;  // Note: pushing stick forward gives negative value
            double x = gamepad1.left_stick_x;
            double rotateX = gamepad1.right_stick_x;

            double lift = -gamepad2.left_stick_y;
            double grabber = -gamepad2.right_stick_y;

            if(lift > .05){
                liftMotor.setPower(1);
            }else{
                liftMotor.setPower(0);
            }
            if(lift < -.05){
                liftMotor.setPower(-.4);
            }else{
                liftMotor.setPower(0);
            }

            if(grabber>.1 || grabber<-.1){
                if(grabber<.05){
                    direction = .1;
                }else{
                    direction = -.1;
                }
            }else{
                direction = 0;
            }

            position+=direction;
            position2+= -direction;

            if(position < MAX_POS || position2 > MAX_POS2){
                position=MAX_POS;
                position2=MAX_POS2;
            }

            if(position > MIN_POS || position2 < MIN_POS2){
                position=MIN_POS;
                position2=MIN_POS2;
            }

            servoGrabber1.setPosition(position);
            servoGrabber2.setPosition(position2);

            // Send calculated power to wheels
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rotateX), 1);
            leftFrontDrive.setPower((y + x + rotateX) / denominator);
            leftBackDrive.setPower((y - x + rotateX) / denominator);
            rightFrontDrive.setPower((y - x - rotateX) / denominator);
            rightBackDrive.setPower((y + x - rotateX) / denominator);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
        }
    }
}