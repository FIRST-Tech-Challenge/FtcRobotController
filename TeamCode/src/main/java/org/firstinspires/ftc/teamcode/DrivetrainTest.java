package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.robotSubSystems.drivetrain.Gyro;
import org.firstinspires.ftc.teamcode.utils.Vector;

@TeleOp
public class DrivetrainTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Drivetrain.init(hardwareMap);

        waitForStart();

        while (!isStopRequested()) {
            final float omega = gamepad1.right_trigger - gamepad1.left_trigger;
            final Vector joystick = new Vector(gamepad1.left_stick_x, -gamepad1.left_stick_y);

//            if (gamepad1.left_stick_button) {
                Drivetrain.operate(joystick, omega);
//            }
//            else {
//                Drivetrain.testMotors(gamepad1);
//            }


            if (gamepad1.dpad_down){
                Gyro.resetGyro();
            }

            telemetry.addData("angle",Gyro.getAngle());
            telemetry.addData("last angle",Gyro.lastAngle);
            telemetry.update();


        }
    }
}
