package org.firstinspires.ftc.teamcode.teleOP;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name = "Wrist Tester")
public class WristTester extends LinearOpMode {
    GamepadEvents controller1;
    Wrist wrist;
    @Override
    public void runOpMode() throws InterruptedException {
        controller1 = new GamepadEvents(gamepad1);
        wrist = new Wrist(hardwareMap, "wrist");

        waitForStart();
        wrist.initPos();
        while(opModeIsActive())
        {
            if(controller1.b.onPress())
            {

                    wrist.setParallel();
            }

            wrist.adjustPosition(-controller1.left_stick_y);


//            if(controller1.b.onPress())
//            {
//
//                wrist.setPosition(0.5);
//            }




            controller1.update();


//            telemetry.addLine("Press [A] to set Parallel");
            telemetry.addLine("Press [B] to set as parallel");
//            telemetry.addLine("Press [X] to set as 1");
            telemetry.addLine("use [Left Joy Stick Y] to adjust positions");
            telemetry.addData("Wrist Pos", wrist.getPosition());

            telemetry.update();
        }
    }
}
