package org.firstinspires.ftc.teamcode.src.DrivePrograms.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.src.Utills.TeleopTemplate;


@TeleOp(name = "2022 Drive Program")
public class JavaDriveProgram extends TeleopTemplate {

    public void runOpMode() throws InterruptedException {

        this.initAll();
        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            driveTrain.setPowerFromGamepad(gamepad1);

            //Handles Linear Slide Control
            slide.setMotorPower(0.75 * gamepad2.left_stick_y);
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
