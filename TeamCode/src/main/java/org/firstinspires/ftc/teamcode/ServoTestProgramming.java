package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
@TeleOp(name="Servo Test", group="Linear OpMode")
@Disabled
public class ServoTestProgramming extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private Servo servo = null;
    private double servopos;

    @Override
    public void runOpMode() {

        servo = hardwareMap.get(Servo.class, "servo");
        servo.setDirection(Servo.Direction.REVERSE);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();


        waitForStart();
        runtime.reset();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            servopos = servo.getPosition();
//            servo.setPosition(-gamepad1.left_stick_y/2);

            if (gamepad1.dpad_up){
                servo.setPosition(0.2);
            } else if (gamepad1.dpad_down) {
                servo.setPosition(0.8);
            } else if (gamepad1.dpad_right) {
                servo.setPosition(servopos + 0.005);
            } else if (gamepad1.dpad_left) {
                servo.setPosition(servopos - 0.005);
            }

            telemetry.addData("Status", "LeftStickY " + -gamepad1.left_stick_y);
            telemetry.addData("Status", "ServoPos " + servopos);
            telemetry.update();
        }
    }}
