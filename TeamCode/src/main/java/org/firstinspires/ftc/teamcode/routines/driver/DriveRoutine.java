package org.firstinspires.ftc.teamcode.routines.driver;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.routines.Routine;
import org.firstinspires.ftc.teamcode.subsystems.DriveSystem;

@TeleOp(name = "Drive")
public class DriveRoutine extends Routine {
    DriveSystem driveSystem;

    @Override
    public void onInit(){
        super.onInit();
        driveSystem = new DriveSystem(this);
    }

    @Override
    public void onStart(){

        while(opModeIsActive()){
            driveSystem.driveOnTick(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);

            telemetry.addData("left_stick_x", gamepad1.left_stick_x);
            telemetry.addData("left_stick_y", gamepad1.left_stick_x);
            telemetry.addData("right_stick_x", gamepad1.right_stick_x);
            telemetry.update();
        }

    }
}