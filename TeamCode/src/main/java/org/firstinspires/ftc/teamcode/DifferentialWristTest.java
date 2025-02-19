package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.bots.DifferentialWristBot;
import org.firstinspires.ftc.teamcode.bots.RollerIntakeBot;

@TeleOp(name = "DifferentialWristTest")
public class DifferentialWristTest extends LinearOpMode {
    private DifferentialWristBot differentialWristBot;

    @Override
    public void runOpMode() throws InterruptedException {
        differentialWristBot = new DifferentialWristBot(this);
        differentialWristBot.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            differentialWristBot.onLoop(0,"test");
//            if(gamepad1.dpad_up){
//                differentialWristBot.pitch(0.3);
//            }
//            if(gamepad1.dpad_down){
//                differentialWristBot.pitch(-0.3);
//            }
//            if(gamepad1.dpad_left){
//                differentialWristBot.roll(0.3);
//            }
//            if(gamepad1.dpad_right){
//                differentialWristBot.roll(-0.3);
//            }
            differentialWristBot.pitchTo(differentialWristBot.tuneTargetPitch
            );
            differentialWristBot.rollTo(differentialWristBot.tuneTargetRoll
            );



            telemetry.update();
        }
    }
}