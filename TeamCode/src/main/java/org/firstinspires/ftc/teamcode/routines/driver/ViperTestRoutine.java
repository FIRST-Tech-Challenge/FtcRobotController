package org.firstinspires.ftc.teamcode.routines.driver;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.routines.Routine;
import org.firstinspires.ftc.teamcode.subsystems.ViperSystem;

@TeleOp(name = "ViperTest")
public class ViperTestRoutine extends Routine {
    public ViperSystem viperSystem;

    @Override
    public void onInit() {
        super.onInit();
        viperSystem = new ViperSystem(this);
    }

    @Override
    public void onStart() {

        while(opModeIsActive()){
            viperSystem.moveOnTick(gamepad1.dpad_up, gamepad1.dpad_down);

            telemetry.addData("Left Viper Motor", viperSystem.getViper_motor_l().getCurrentPosition());
            telemetry.addData("Right Viper Motor", viperSystem.getViper_motor_r().getCurrentPosition());
            telemetry.update();
        }

    }
}
