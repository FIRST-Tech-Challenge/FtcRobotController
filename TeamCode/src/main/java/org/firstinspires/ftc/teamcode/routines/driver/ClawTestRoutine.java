package org.firstinspires.ftc.teamcode.routines.driver;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.routines.Routine;
import org.firstinspires.ftc.teamcode.subsystems.ClawSystem;

@TeleOp(name = "ClawTest")
public class ClawTestRoutine extends Routine {
    public ClawSystem clawSystem;

    @Override
    public void onInit() {
        super.onInit();
        clawSystem = new ClawSystem(this);
    }

    @Override
    public void onStart() {

        while(opModeIsActive()){
            clawSystem.toggleClaw(gamepad1.b);
            telemetry.addData("Left Servo Claw", clawSystem.getClaw_servo_l().getPosition());
            telemetry.addData("Right Servo Claw", clawSystem.getClaw_servo_r().getPosition());
            telemetry.update();
        }

    }
}
