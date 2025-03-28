package org.firstinspires.ftc.teamcode.teleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Claw;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name = "TSA Claw Tester")
public class ClawTester extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Claw claw = new Claw(hardwareMap,"Claw");
        GamepadEvents controller = new GamepadEvents(gamepad1);

        waitForStart();
        while(opModeIsActive())
        {
            if(controller.x.onPress())
            {
                claw.close();
            }else if(controller.y.onPress()){
                claw.open();
            }else if(controller.b.onPress()){
                claw.ringClose();
            }
            claw.adjustPosition(controller.left_stick_y);

            telemetry.addData("Expected Claw Pos", claw.getPos());
            telemetry.update();
            controller.update();
        }
    }
}
