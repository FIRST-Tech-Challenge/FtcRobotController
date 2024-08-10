package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Continuous Servo Test", group="Linear OpMode")
@Disabled
public class ContinuousServoTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private CRServo servo = null;
    private double speed = .25;
    private double power = 0;
    @Override
    public void runOpMode() {

        servo = hardwareMap.get(CRServo.class, "servo");
        servo.setDirection(CRServo.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();



        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            if (gamepad1.left_bumper){
                speed = .125;
            } else if (gamepad1.right_bumper) {
                speed = .5;
            }else{
                speed = .25;
            }


            power = -gamepad1.left_stick_y * speed;
            servo.setPower(power);

            telemetry.addData("Status", "ServoPos " + servo.getPower());
            telemetry.addData("Status", "Cross Pressed " + gamepad1.cross);
            telemetry.addData("Status", "Square Pressed " + gamepad1.square);
            telemetry.addData("Status", "Triangle Pressed " + gamepad1.triangle);
            telemetry.addData("Status", "Circle Pressed " + gamepad1.circle);
            telemetry.update();

        }
    }}
