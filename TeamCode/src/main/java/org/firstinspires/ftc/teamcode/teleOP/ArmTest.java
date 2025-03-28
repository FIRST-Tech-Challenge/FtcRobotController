package org.firstinspires.ftc.teamcode.teleOP;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name = "Arm Test")
public class ArmTest extends LinearOpMode {
    GamepadEvents controller1;
    Arm arm;
    Wrist wrist;


    @Override
    public void runOpMode() throws InterruptedException {
        controller1 = new GamepadEvents(gamepad1);
        arm = new Arm(hardwareMap, "frontArm", "backArm");
        wrist = new Wrist(hardwareMap, "wrist");
        waitForStart();
        wrist.initPos();
        while(opModeIsActive()){
//            if(gamepad1.a){
//                arm.setPosition(0.5);
//            }
//            if(gamepad1.b){
//                arm.setPosition(0.5);
//            }
            if(gamepad1.a){
                arm.setParallel();
            }
            if(gamepad1.b){
                wrist.setParallel();
            }

            arm.adjustPosition(controller1.left_stick_y);
            wrist.adjustPosition(controller1.right_stick_y);
            telemetry.addLine(arm.toString());
            telemetry.addLine(wrist.toString());
            telemetry.update();
            controller1.update();
        }
    }
}
