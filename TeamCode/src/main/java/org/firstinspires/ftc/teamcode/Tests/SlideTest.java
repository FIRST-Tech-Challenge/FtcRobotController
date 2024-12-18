package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "SlideTest")
public class SlideTest extends LinearOpMode {
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

            double diameterBig = 28.8; //mm
            double diameterSmall = 7; //mm

            double speed = 0;
            boolean pressY = false;
            boolean pressA = false;
            boolean pressB = false;
            double deg = 0;
            while (opModeIsActive()) {
                pos = motor.getCurrentPosition();
                deg = (360 / 751.8) * pos * (28.8 / 7);

                speed = 0;
                if (gamepad1.y && !pressY) { //if pressing b and var is false
                    speed += 0.1;
                    pressY = true;
                    motor.setPower(speed);
                } else if (!gamepad1.y && pressY) {
                    pressY = false;
                }


                if (gamepad1.a && !pressA) {
                    speed -= 0.1;
                    pressA = true;
                    motor.setPower(speed);
                } else if (!gamepad1.a && pressA) {
                    pressA = false;
                }


                if (pos > 2000 || pos < -20) {
                    motor.setPower(0);
                }


                if (gamepad1.b && !pressB) { //if pressing b and var is false
                    motor.setPower(0);
                }

                telemetry.addData("motor speed", speed);
                telemetry.addData("Pos: ", pos);
                telemetry.addData("Degrees: ", deg);

                telemetry.update();
            }
        }
    }
}




    //slide test : max pos 3512
    // degree = 6919.0925

// min pos -29 , degree -57
