package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="tank_drive", group="Linear OpMode")
// @Disabled
public class tank_drive extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    public DcMotor leftwheel = null;
    public DcMotor rightwheel = null;
    public Servo grabber;
    public Servo tilt;
    public double tilt_position;
    public Servo arm;
    public double arm_position;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables.
        leftwheel  = hardwareMap.get(DcMotor.class, "leftwheel");
        rightwheel = hardwareMap.get(DcMotor.class, "rightwheel");
        grabber  = hardwareMap.get(Servo.class, "grabber");
        tilt = hardwareMap.get(Servo.class, "tilt");
        arm = hardwareMap.get(Servo.class, "arm");

        // Set motor directions.
        leftwheel.setDirection(DcMotor.Direction.REVERSE);
        rightwheel.setDirection(DcMotor.Direction.FORWARD);
        grabber.setDirection(Servo.Direction.FORWARD);
        tilt.setDirection(Servo.Direction.FORWARD);
        arm.setDirection(Servo.Direction.FORWARD);

        // Wait for the game to start (driver presses START).
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup variables for motor power
            double leftPower;
            double rightPower;

            tilt.setPosition(0.5);
            grabber.setPosition(0.5);
            arm.setPosition(0.3);

            // Map the joystick inputs to motor power
            leftPower  = -gamepad1.left_stick_y;
            rightPower = -gamepad1.right_stick_y;

            // Send calculated power to wheels
            leftwheel.setPower(leftPower);
            rightwheel.setPower(rightPower);

            while (gamepad1.y) {
                tilt_position = tilt.getPosition();
                tilt.setPosition(tilt_position + 0.03);
            }

            while (gamepad1.a) {
                tilt_position = tilt.getPosition();
                tilt.setPosition(tilt_position - 0.03);
            }

            if (gamepad1.x) {
                grabber.setPosition(0);
            }

            if (gamepad1.b) {
                grabber.setPosition(1);
            }

            while (gamepad1.right_bumper) {
                arm_position = arm.getPosition();
                arm.setPosition(arm_position + 0.03);
            }

            while (gamepad1.right_bumper) {
                arm_position = arm.getPosition();
                arm.setPosition(arm_position - 0.03);
            }

            // Telemetry to display key data
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Left Motor Power", leftPower);
            telemetry.addData("Right Motor Power", rightPower);
            telemetry.addData("tilt Servo Position", tilt.getPosition());
            telemetry.addData("grabber Servo Position", grabber.getPosition());
            telemetry.addData("arm Servo Position", arm.getPosition());
            telemetry.update();
        }
    }
}