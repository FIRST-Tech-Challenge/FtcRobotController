package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.common.Constants;

@TeleOp(name="science to be done", group="Linear OpMode")
public class BasicOmniOpMode_LinearTest extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private DcMotor intake = null;
    private DcMotorEx lift = null;
    private Servo servo = null;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        intake = hardwareMap.get(DcMotor.class, "intake");
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        lift.setDirection(DcMotorSimple.Direction.REVERSE);
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
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
            int liftPos = lift.getCurrentPosition();
            telemetry.addData("lift position: ", liftPos);
            double liftPower = 0;
            if (liftPos > Constants.TopLiftPosition && -gamepad2.right_stick_y > 0) liftPower = gamepad2.right_stick_y;
            else if (liftPos < Constants.IntakingLiftPosition && -gamepad2.right_stick_y < 0) liftPower = gamepad2.right_stick_y;
            else liftPower = gamepad2.right_stick_y;
            lift.setVelocity(liftPower * 1500 + 1);
            intake.setPower(-gamepad2.right_trigger);
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
