package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Pivot;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name = "Pivot Tester")
public class PivotTester extends LinearOpMode {
    private Pivot pivot;
    private GamepadEvents controller;
    private int i = 100;
    private int pos = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        pivot = new Pivot(hardwareMap, "pivot", "pivot");
        controller = new GamepadEvents(gamepad1);

        telemetry.addLine("Initalized");
        waitForStart();
        while(opModeIsActive())
        {
//            if(controller.left_bumper.onPress())
//            {
//                pos += i;
//                pivot.setPosition(pos);
//            }else if(controller.right_bumper.onPress())
//            {
//                pos -= i;
//                pivot.setPosition(pos);
//            }
            pivot.toggle();
            telemetry.addData("Pivot Pos", pivot.getPosition());
            telemetry.update();
            controller.update();
        }
    }
}
