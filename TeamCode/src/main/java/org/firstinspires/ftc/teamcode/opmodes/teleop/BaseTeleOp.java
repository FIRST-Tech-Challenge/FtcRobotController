package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static org.firstinspires.ftc.teamcode.components.ArmSystem.Intake.State.INTAKING;
import static org.firstinspires.ftc.teamcode.components.ArmSystem.Intake.State.OUTTAKING;

import org.firstinspires.ftc.teamcode.components.ArmSystem;
import org.firstinspires.ftc.teamcode.opmodes.base.BaseOpMode;

@TeleOp(name = "Base TeleOp", group="TeleOp")
public class BaseTeleOp extends BaseOpMode {

    // private ArmSystem backdrop =

    public void loop() {
        float rx = (float) Math.pow(gamepad2.right_stick_x, 3);
        float lx = (float) Math.pow(gamepad2.left_stick_x, 3);
        float ly = (float) Math.pow(gamepad2.left_stick_y, 3);

        telemetry.addData("Right X", Math.pow(gamepad2.right_stick_x, 3));
        telemetry.addData("Left Y", Math.pow(gamepad2.left_stick_y, 3));

        //idk what else to put because like roadrunner not done !
    }
}
