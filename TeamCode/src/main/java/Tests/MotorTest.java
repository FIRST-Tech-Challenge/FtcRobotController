package Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "MotorTest")
public class MotorTest extends LinearOpMode {
    DcMotorEx motor;
    double pos;

    @Override
    public void runOpMode() throws InterruptedException {

        motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); //if you leave in this mode it won't move

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER); //need this line when you use the above line
        // or it won't move

        waitForStart();

        while (opModeIsActive()) {

            double speed = 0;
            boolean pressY = false;
            boolean pressA = false;
            while (opModeIsActive()) {
                pos = motor.getCurrentPosition();
                double degrees = (360 / 751.8) * pos;
                if (gamepad1.y && !pressY) { //if pressing b and var is false
                    speed += 0.1;
                    motor.setPower(speed);
                    pressY = true;
                } else if (!gamepad1.y && pressY) {
                    pressY = false;
                }

                if (gamepad1.a && !pressA) {
                    speed -= 0.1;
                    motor.setPower(speed);
                    pressA = true;
                } else if (!gamepad1.a && pressA) {
                    pressA = false;
                }

                telemetry.addData("motor speed", speed);
                telemetry.addData("Pos: ", pos);
                telemetry.addData("Degrees: ", degrees);
                telemetry.update();
            }
        }
    }
}
