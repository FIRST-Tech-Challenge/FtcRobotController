package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="MotorManualTeleop", group="A")

public class MotorManualTeleop extends DriveMethods{
    DcMotor motor;
    double motorPower = 0;
    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotor.class, "motor");

        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        waitForStart();


        while(opModeIsActive()){
            if(gamepad2.dpad_up){
                motorPower = motorPower + 0.01;
                sleep(100);
            }
            if(gamepad2.dpad_down){
                motorPower = motorPower - 0.01;
                sleep(100);

            }

            if(gamepad2.dpad_right){
                motorPower = motorPower + 0.05;
                sleep(100);

            }
            if(gamepad2.dpad_left){
                motorPower = motorPower - 0.05;
                sleep(100);

            }

            motor.setPower(motorPower);
            telemetry.addLine("Motorpower: " + motorPower);
            telemetry.addLine("Motorposition: " + motor.getCurrentPosition());
            telemetry.update();
        }
    }
}

/*

Giraffe

                          _- _ , - . _
                        `,% o` ~~-_,'.'
                        % %@ - % %, -'%,
                       ,-, . _ --\ -.%
              P^=.     `'"   |+|'    `
              ||             |+|
              ||             |+|
              ||             |+|
        ______/|             |+|
       `| ___ ,/             |+|
        ||   ||              |+|
        ||   ||              |+|
________||___||___.__________/H|____




 */