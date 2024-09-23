package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops;

import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.EasyGamepad;

public class Test extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Intake intake = new Intake(hardwareMap);
        EasyGamepad easyGamepad1 = new EasyGamepad(gamepad1);
        EasyGamepad easyGamepad2 = new EasyGamepad(gamepad2);
        while (opModeIsActive()){
            easyGamepad1.update(gamepad1);
            easyGamepad2.update(gamepad2);
            Actions.runBlocking(
                    intake.intake()
            );
        }
    }
}
