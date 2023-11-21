package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.common.Button;
import org.firstinspires.ftc.teamcode.common.Constants;

@TeleOp(name="science to be done", group="Linear OpMode")
public class BasicOmniOpMode_LinearTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private DcMotor intake = null;
    private Servo servo = null;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        intake = hardwareMap.get(DcMotor.class, "intake");
        servo = hardwareMap.get(Servo.class, "servo");
        double servoSetpoint = 0;
        // Wait for the game to start (driver presses PLAY)
        while (!isStarted() && !isStopRequested()) {
            telemetry.addLine("kjdfkjdk");
            telemetry.update();
        }
        // servo position 0.18 is good for some reason

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            intake.setPower(gamepad2.right_stick_y);
            telemetry.addData("gamepad2 y:" , gamepad2.y);
            telemetry.addData("gamepad2 dpad up: ", gamepad2.dpad_up);
            telemetry.addData("gamepad2 right joystick y: ", gamepad2.right_stick_y);
            if (gamepad2.a) {
                servoSetpoint = 0.13;// 0.1 worked aright 0.13 works better
                telemetry.addLine("Decrementing servo setpoint");
            }
            else {
                telemetry.addLine("Incrementing servo setpoint");
                servoSetpoint = 0.00;
            }
            sleep(20);
            telemetry.addData("servoSetpoint: ", servoSetpoint);
            telemetry.addData("Last servo setpoint: ", servo.getPosition());
            if (servo.getPosition() != servoSetpoint) servo.setPosition(servoSetpoint);
            telemetry.update();
        }
    }
    public void score() {
        
    }
}
