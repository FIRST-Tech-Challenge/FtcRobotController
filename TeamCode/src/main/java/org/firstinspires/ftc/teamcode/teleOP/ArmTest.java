package org.firstinspires.ftc.teamcode.teleOP;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

public class ArmTest extends LinearOpMode {
    GamepadEvents controller1;
    Arm arm;
    @TeleOp(name = "Arm Test")
    public void runOpMode() throws InterruptedException {
        controller1 = new GamepadEvents(gamepad1);
        arm = new Arm(hardwareMap, "frontArm", "backArm");
        waitForStart();
        @Override
        while(opModeIsActive()){
            if(gamepad1.a){
                arm.setPosition(0.1);
            }
            if(gamepad1.b){
                arm.setPosition(-0.1);
            }
        }
    }
}
