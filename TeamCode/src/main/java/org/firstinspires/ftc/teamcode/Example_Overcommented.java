package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@TeleOp(name = "Drive with this one")
public class Example_Overcommented extends LinearOpMode {
    final double DESIRED_DISTANCE = 12.0;
    final double SPEED_GAIN = 0.02;   //Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN = 0.015;   //Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN = 0.01;   //Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)

    final double MAX_AUTO_SPEED = 0.5;   //Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_STRAFE = 0.5;   //Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN = 0.3;
    private static final int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    private AprilTagDetection desiredTag = null;
    private DcMotor motorFrontLeft = null;  //  Used to control the left front drive wheel
    private DcMotor motorFrontRight = null;  //  Used to control the right front drive wheel
    private DcMotor motorBackLeft = null;  //  Used to control the left back drive wheel
    private DcMotor motorBackRight = null;//  Used to control the right back drive wheel
    private DcMotor hang = null;
    Servo servo;
    static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_POS = 2.0;     // Maximum rotational position
    static final double MIN_POS = 0.0;     // Minimum rotational position

    //Define class members
    double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    boolean rampUp = true;
    boolean retract = false;

    @Override
    public void runOpMode() throws InterruptedException {
        boolean targetFound = false;    // Set to true when an AprilTag target is detected
        double drive = 0;        // Desired forward power/speed (-1 to +1)
        double strafe = 0;        // Desired strafe power/speed (-1 to +1)
        double turn = 0;        // Desired turning power/speed (-1 to +1)

        //initAprilTag();


        //Uses [Z] Motor Config
        //These lines get the motors from the configuration and assign them to variables and may need to
        //Be changed to fit different robots

        motorFrontLeft = hardwareMap.dcMotor.get("frontRight"); //mapping all the motors
        motorFrontRight = hardwareMap.dcMotor.get("frontLeft");
        motorBackLeft = hardwareMap.dcMotor.get("backLeft");
        motorBackRight = hardwareMap.dcMotor.get("backRight");
        hang = hardwareMap.dcMotor.get("hang");
        servo = hardwareMap.get(Servo.class, "left_hand");

        double FrontLeftMotorLastSpeed = 0; 
        double FrontRightMotorLastSpeed = 0;
        double BackLeftMotorLastSpeed = 0;
        double BackRightMotorLastSpeed = 0;


        double Speed = 1;
        int Delay = 0;

        // Reverse the right side motors
        // This may or may not need to be changed based on how the robots motors are mounted
        // If movement is weird mess with these first
        motorBackRight.setDirection(DcMotorSimple.Direction.REVERSE);
        //hang.setDirection(DcMotorSimple.Direction.FORWARD);

        // This is the line that ends the init of the bot
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            // These lines assign game-pad 1 joysticks to variables
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.right_stick_x;
            double rx = gamepad1.left_stick_x;

            // This makes variables for the motor power and sets it based on some math
            // That takes the joystick x and y and does some things for motor power
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            // Testing Spinny
            if (gamepad1.b) {
                position += INCREMENT;
                if (position >= MAX_POS) {
                    position = MAX_POS;
                }
            }

            if (gamepad1.a) {
                position -= INCREMENT;
                if (position <= MIN_POS) {
                    position = MIN_POS;
                }
            }
            telemetry.addData("Servo Position", "%5.2f", position);
            if (gamepad1.x) {
                hang.setPower(0.5);
            }

            if (gamepad1.y) {
                hang.setPower(-0.5);
            }
            if (gamepad1.right_bumper) {
                hang.setPower(0);
            }

            // Adds telemetry on the control hub to check stick positions
            telemetry.addData("Gamepad X", x);
            telemetry.addData("Gamepad Y", y);

            //if(gamepad1.left_trigger > 0){
            //telemetry.addData("Send Drone", true);
            //motorDroneSend.setPower(1);
            //}

            // Sends it to the control hub
            telemetry.update();


            // Sets the motor powers to whatever was decided on in the top math
            motorFrontLeft.setPower(frontLeftPower * 0.7);
            motorBackLeft.setPower(backLeftPower * 0.7);
            motorFrontRight.setPower(frontRightPower * 0.7);
            motorBackRight.setPower(backRightPower * 0.7);
            servo.setPosition(position);
        }
    }
}
