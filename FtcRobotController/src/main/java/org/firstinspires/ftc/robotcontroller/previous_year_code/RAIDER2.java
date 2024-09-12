package org.firstinspires.ftc.robotcontroller.previous_year_code;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="RAIDER2", group="Linear OpMode")
public class RAIDER2 extends LinearOpMode {
    //private Gyroscope imu;
    //private DcMotor motorTest;
    //private DigitalChannel digitalTouch;
    //private DistanceSensor sensorColorRange;
    //private Servo servoTest;

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor armMotor = null;
    private Servo wristMotor = null;
    private Servo clawMotor1 = null;
    private Servo clawMotor2 = null;
    private Servo slingshotRelease = null;
    private DcMotor hangMotor = null;

    @Override
    public void runOpMode() {
        //imu = hardwareMap.get(Gyroscope.class, "imu");
        //motorTest = hardwareMap.get(DcMotor.class, "motorTest");
        //digitalTouch = hardwareMap.get(DigitalChannel.class, "digitalTouch");
        //sensorColorRange = hardwareMap.get(DistanceSensor.class, "sensorColorRange");
        //servoTest = hardwareMap.get(Servo.class, "servoTest");

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "left_front_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "left_back_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "right_front_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "right_back_drive");
        armMotor = hardwareMap.get(DcMotor.class, "arm_Motor");
        wristMotor = hardwareMap.get(Servo.class, "wrist_Motor");
        clawMotor1 = hardwareMap.get(Servo.class, "claw_Motor1");
        clawMotor2 = hardwareMap.get(Servo.class, "claw_Motor2");
        slingshotRelease = hardwareMap.get(Servo.class, "slingshot_Release");
        hangMotor = hardwareMap.get(DcMotor.class, "hang_Motor");

        leftFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        armMotor.setDirection(DcMotor.Direction.REVERSE);
        wristMotor.setDirection(Servo.Direction.REVERSE);
        clawMotor1.setDirection(Servo.Direction.FORWARD);
        clawMotor2.setDirection(Servo.Direction.REVERSE);
        slingshotRelease.setDirection(Servo.Direction.FORWARD);
        hangMotor.setDirection(DcMotor.Direction.FORWARD);

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        armMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hangMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        double wristMotorPower = 1.0;
        double clawMotorPower = 0.0;
        double slingshotPosition = 0.0;

        telemetry.addData("Status", "Initialized");
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            double max;
            double slingshotChanger = 0.0;
            double clawMotorChanger = 0.0;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double axial;  // Note: pushing stick forward gives negative value
            double lateral;
            double yaw;
            double armPitch = gamepad2.left_stick_y;
            double wristPitch = gamepad2.right_stick_y;
            double positiveHangMotorPower = gamepad2.right_trigger;
            double negativeHangMotorPower = gamepad2.left_trigger;

            if (gamepad2.b){
                slingshotPosition = 1.0;
            } else if (gamepad2.a) {
                slingshotPosition = 0.0;
            } else {
                slingshotPosition = 0.0;
            }


            if (gamepad2.right_bumper){
                clawMotorPower = 1.0;
            } else if (gamepad2.left_bumper){
                clawMotorPower = 0.0;
            } else {
                clawMotorPower = 0.0;
            }




            if (gamepad1.y) {
                axial = 1.0;
            }
            else if (gamepad1.a) {
                axial = -1.0;
            } else {
                axial = 0.0;
            }

            if (gamepad1.x) {
                lateral = -1.0;
            }
            else if (gamepad1.b) {
                lateral = 1.0;
            } else {
                lateral =  0.7 * gamepad1.left_stick_x;
            }

            if (gamepad1.dpad_up) {
                axial = 0.1;
            }
            else if (gamepad1.dpad_down) {
                axial = -0.1;
            } else {
                axial = -0.7 * gamepad1.left_stick_y;
            }

            if (gamepad1.dpad_left) {
                yaw = 0.1;
            }
            else if (gamepad1.dpad_right) {
                yaw = -0.1;
            } else {
                yaw   =  -0.7 * gamepad1.right_stick_x;
            }

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double leftFrontPower  = axial + lateral + yaw;
            double rightFrontPower = axial - lateral - yaw;
            double leftBackPower   = axial - lateral + yaw;
            double rightBackPower  = axial + lateral - yaw;
            double armMotorPower = 0.70 * armPitch;
            double hangMotorPower = positiveHangMotorPower - negativeHangMotorPower;
            wristMotorPower += 0.001 * wristPitch;
            clawMotorPower += clawMotorChanger;
            slingshotPosition += slingshotChanger;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
            max = Math.max(max, Math.abs(leftBackPower));
            max = Math.max(max, Math.abs(rightBackPower));
            max = Math.max(max, Math.abs(armMotorPower));
            max = Math.max(max, Math.abs(wristMotorPower));
            max = Math.max(max, Math.abs(clawMotorPower));
            max = Math.max(max, Math.abs(slingshotPosition));
            max = Math.max(max, Math.abs(hangMotorPower));

            if (max > 1.0) {
                leftFrontPower  /= max;
                rightFrontPower /= max;
                leftBackPower   /= max;
                rightBackPower  /= max;
                armMotorPower   /= max;
                wristMotorPower /= max;
                clawMotorPower  /= max;
                slingshotPosition /= max;
                hangMotorPower /= max;
            }

            if (clawMotorPower < 0.0){
                clawMotorPower = 0.0;
            }

            if (slingshotPosition < 0.0){
                slingshotPosition = 0.0;
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
            leftFrontDrive.setPower(leftFrontPower);
            rightFrontDrive.setPower(rightFrontPower);
            leftBackDrive.setPower(leftBackPower);
            rightBackDrive.setPower(rightBackPower);
            armMotor.setPower(armMotorPower);
            wristMotor.setPosition(wristMotorPower);
            clawMotor1.setPosition(clawMotorPower);
            clawMotor2.setPosition(clawMotorPower);
            slingshotRelease.setPosition(slingshotPosition);
            hangMotor.setPower(hangMotorPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", leftFrontPower, rightFrontPower);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", leftBackPower, rightBackPower);
            telemetry.addData("Arm", "%4.2f", armMotorPower);
            telemetry.addData("Wrist", "%4.2f", wristMotorPower);
            telemetry.addData("Wrist Changer", "%4.2f", wristPitch);
            telemetry.addData("Claw", "%4.2f", clawMotorPower);
            telemetry.addData("Claw Changer", "%4.2f", clawMotorChanger);
            telemetry.addData("Slingshot", "%4.2f", slingshotPosition);
            telemetry.addData("Slingshot Changer", "%4.2f", slingshotChanger);
            telemetry.addData("HangMotor", "%4.2f", hangMotorPower);
            telemetry.update();
        }
    }}
