package org.firstinspires.ftc.teamcode.routines.driver;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.routines.Routine;
import org.firstinspires.ftc.teamcode.subsystems.clawSystem;

@TeleOp(name = "ClawTest")
public class ClawTestRoutine extends Routine {
    public ClawSystem clawSystem;

    @Override
    public void onInit() {
        super.onInit();
        clawSystem = new clawSystem(this);
    }

    @Override
    public void onStart() {

        while(opModeIsActive()){
            clawSystem.moveOnTick(gamepad1.dpad_left, gamepad1.dpad_right);

            telemetry.addData("Left claw Motor", clawSystem.getclaw_motor_l().getCurrentPosition());
            telemetry.addData("Right claw Motor", clawSystem.getclaw_motor_r().getCurrentPosition());
            telemetry.update();
        }

    }
}
