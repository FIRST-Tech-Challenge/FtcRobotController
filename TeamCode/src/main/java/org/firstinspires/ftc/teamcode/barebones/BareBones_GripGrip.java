package org.firstinspires.ftc.teamcode.barebones;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

/**
 */
@TeleOp(name="BareBones: Grip", group="BareBones")
public class BareBones_GripGrip extends OpMode {

    // Instance Members.
    private boolean doMotors = true;
    private boolean doArm = true;

    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor arm;
    private Servo fingers;
    double servoStart = 0.2;


    // Called once, right after hitting the Init button.
    @Override
    public void init() {


        if (doMotors) {
            // Initialize Motors, finding them through the hardware map.
            leftDrive = hardwareMap.get(DcMotor.class, "motorLeft");
            rightDrive = hardwareMap.get(DcMotor.class, "motorRight");

            leftDrive.setDirection(DcMotor.Direction.REVERSE);
            rightDrive.setDirection(DcMotor.Direction.FORWARD);

            // Set all motors to zero power.
            leftDrive.setPower(0);
            rightDrive.setPower(0);


            // BC: set the servo to  a known position.

            // Set all motors to run without encoders.
            // May want to use RUN_USING_ENCODERS if encoders are installed.
            leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addData("Yo", "Initialized Drive, motors=%b", doMotors);

        if(doArm) {
            // Initialize Motors, finding them through the hardware map.
            arm = hardwareMap.get(DcMotor.class, "arm");
            fingers = hardwareMap.get(Servo.class, "fingers");

            arm.setDirection(DcMotor.Direction.FORWARD);
            fingers.setDirection(Servo.Direction.FORWARD);

            // Set all motors to zero power.
            arm.setPower(0);
            fingers.setPosition(servoStart);

            // Set all motors to run without encoders.
            arm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        telemetry.addData("Yo", "Initialized Drive, arms=%b", doArm);
    }

    // Called repeatedly, right after hitting start, up until hitting stop.
    @Override
    public void loop() {

        // BC: seperate arm and wheel code
        if (doMotors) {
            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.left_stick_y;
            double turn = gamepad1.right_stick_x;

            double leftPower = Range.clip(drive + turn, -1.0, 1.0);
            double rightPower = Range.clip(drive - turn, -1.0, 1.0);

            // Send calculated power to wheels
            leftDrive.setPower(leftPower);
            rightDrive.setPower(rightPower);
            telemetry.addData("left", "%.2f", leftPower);
            telemetry.addData("right", "%.2f", rightPower);


        }

        if(doArm) {
            boolean lift;
            boolean drop;
            boolean grip;

            lift = gamepad1.y;
            drop = gamepad1.a;

            grip = gamepad1.left_bumper;

            // ARM LIFTING CODE
            if(lift) {
                arm.setPower(0.5);
            } else if(drop) {
                arm.setPower(-0.5);
            } else {
                arm.setPower(0);
            }
            telemetry.addData("arm", "%b, %b", lift, drop);

            //GRIPPER OPEN/CLOSE CODE
            if(grip) {
                fingers.setPosition(0.8);
            } else {
                fingers.setPosition(servoStart);
            }
        }

        telemetry.addData("Status", "Run Clock: %.2f", getRuntime());
    }
}
