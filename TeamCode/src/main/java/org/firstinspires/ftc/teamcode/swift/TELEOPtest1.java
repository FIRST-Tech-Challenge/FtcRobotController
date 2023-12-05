package org.firstinspires.ftc.teamcode.swift;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.shared.MotionHardware;
@TeleOp
public class TELEOPtest1 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    static final double INCREMENT = 0.01;     // amount to ramp motor each CYCLE_MS cycle
    static final int CYCLE_MS = 50;     // period of each cycle
    static final double MAX_FWD = 1.0;     // Maximum FWD power applied to motor
    static final double MAX_REV = -1.0;     // Maximum REV power applied to motor
    private Servo leftGripper;
    private Servo rightGripper;
    private Servo launcherServo = null;
    private Servo DroneCoverServo = null;
    private DcMotor armMotor = null;
    MotionHardware robot = new MotionHardware(this);


    @Override
    public void runOpMode() {
        robot.init();

        waitForStart();

        // Define class members
        DcMotor motor;
        double power = 0;
        boolean rampUp = true;
        //Define variables for arm, wrist, and gripper
        Servo wristServo, leftGripper, rightGripper;
        DcMotor armMotor;
        //CRServo contServo;
        float leftY, rightY;
        double armPosition, gripPosition, contPower;
        double MIN_POSITION = 0, MAX_POSITION = 1;
        // called when init button is  pressed.


        armMotor = hardwareMap.get(DcMotor.class, "armMotor");
        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        wristServo = hardwareMap.servo.get("wristServo");
        launcherServo = hardwareMap.get(Servo.class, "launcherServo");
        DroneCoverServo = hardwareMap.get(Servo.class, "DroneCoverServo");
        leftGripper = hardwareMap.get(Servo.class, "leftGripper");
        rightGripper = hardwareMap.get(Servo.class, "rightGripper");
        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        // check the gamepad buttons and if pressed, increment the appropriate position
        // variable to change the servo location.


        telemetry.addData("Mode", "waiting");

        while (opModeIsActive()) {
            telemetry.addData("Mode", "running");
            // check to see if we need to move the servo.
            if (gamepad1.left_bumper) {
                // move to 0 degrees.
                launcherServo.setPosition(0.3);
            } else if (gamepad1.right_bumper || gamepad1.left_bumper) {
                // move to 90 degrees.
                launcherServo.setPosition(1);
            }
            telemetry.addData("Servo Position", launcherServo.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();

            telemetry.addData("Mode", "running");
            // check to see if we need to move the servo.
            if (gamepad1.dpad_left) {
                // move to 0 degrees.
                DroneCoverServo.setPosition(0.3);
            } else if (gamepad1.dpad_right) {
                // move to 90 degrees.
                DroneCoverServo.setPosition(.7);
            }
            telemetry.addData("Servo Position", DroneCoverServo.getPosition());
            telemetry.addData("Status", "Running");
            telemetry.update();

            // Declare Motors

            double y = gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double x = -gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 2);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);

            //start Asher
            // move arm down on A button if not already at lowest position.

            leftY = gamepad2.left_stick_y * -1;
            rightY = gamepad2.right_stick_y * -1;
            armMotor.setPower(Range.clip(leftY, -1.0, 1.0));
            telemetry.addData("sticks", "  left=" + leftY + "  right=" + rightY);

            if (gamepad2.left_bumper) {
                // Move servos in opposite directions when "y" is pressed
                leftGripper.setPosition(1.5); // Adjust the position value as needed

            } else if (gamepad2.left_trigger>0.5) {
                // Return servos to the center position when "x" is pressed
                leftGripper.setPosition(0.95); // Adjust the position value for the center position
            }
                if (gamepad2.right_bumper) {
                    // Move servo in opposite directions when "y" is pressed
                    rightGripper.setPosition(-0.5); // Adjust the position value as needed

                } else if (gamepad2.right_trigger>0.5) {
                    // Return servos to the center position when "x" is pressed
                    rightGripper.setPosition(0.05); // Adjust the position value for the center position
                }
                if (gamepad2.a) {
                    wristServo.setPosition(0.7); // Adjust the position value as needed
                } else if (gamepad2.b) {
                    wristServo.setPosition(0.3); // Adjust the position value for the center position
                }
                if (gamepad2.y) {
                   // robot.moveArm(1, 90, 5);
                    armMotor.setTargetPosition(-4185);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    runtime.reset();
                    armMotor.setPower(Math.abs(1));
                    while (opModeIsActive() &&
                            (runtime.seconds() < 5) &&
                            (armMotor.isBusy()));
                    armMotor.setPower(0);

                    armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                }
                else if (gamepad2.x) {
                    //robot.moveArm(1, -90, 5);
                    armMotor.setTargetPosition(1);
                    armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    runtime.reset();
                    armMotor.setPower(Math.abs(1));
                    while (opModeIsActive() &&
                            (runtime.seconds() < 5) &&
                            (armMotor.isBusy()));
                    armMotor.setPower(0);

                    armMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
                }
            telemetry.update();
            }
        }
    }



