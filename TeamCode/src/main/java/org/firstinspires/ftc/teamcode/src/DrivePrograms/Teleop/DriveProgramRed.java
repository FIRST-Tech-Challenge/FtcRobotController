package org.firstinspires.ftc.teamcode.src.DrivePrograms.Teleop;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.Utills.TeleopTemplate;


@TeleOp(name = "Drive Program Red")
public class DriveProgramRed extends TeleopTemplate {

    public void runOpMode() throws InterruptedException {

        this.initAll();
        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            driveTrain.setPowerFromGamepad(gamepad1);

            //Handles Linear Slide Control
            slide.setMotorPower(1 * gamepad2.left_stick_y);
            intake.setMotorPower(gamepad2.right_trigger - gamepad2.left_trigger);

            if (gamepad2.x){
                spinner.setPowerBlueDirection();
            }else if (gamepad2.b){
                spinner.setPowerRedDirection();
            }else{
                spinner.stop();
            }

            if (gamepad1.b) {
                driveTrain.setDrivePowerMult(0.3);
            }
            if (gamepad1.x) {
                driveTrain.setDrivePowerMult(1);

            }
            if (gamepad1.a) {
                driveTrain.setDrivePowerMult(0.6);
            }
        }
    }
}
