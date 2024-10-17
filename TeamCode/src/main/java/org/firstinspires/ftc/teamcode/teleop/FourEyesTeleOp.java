package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.FourEyesRobot;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

public class FourEyesTeleOp extends LinearOpMode {
    FourEyesRobot fourEyesRobot;
    GamepadEvents controller1,controller2;
    public static GamepadEvents.GamepadButton[] binding1;
    public static GamepadEvents.GamepadButton[] binding2;

    @Override
    public void runOpMode() throws InterruptedException {

        fourEyesRobot  = new FourEyesRobot(hardwareMap);
        controller1 = new GamepadEvents(gamepad1);
        controller2 = new GamepadEvents(gamepad2);
        binding1 = new GamepadEvents.GamepadButton[] {
                null ,
                controller1.a,
                controller1.b
        };
        binding2 = new GamepadEvents.GamepadButton[] {
                null,
                controller2.a,
                controller2.b
        };


        waitForStart();
        while (opModeIsActive()) {
            if(binding1[0].onPress() || binding2[0].onPress()) {
                fourEyesRobot.toggleClaw();
            }
            if(binding1[1].onPress() || binding2[1].onPress()) {
                if(fourEyesRobot.isIntaking()) {
                    fourEyesRobot.intakeForward();
                } else {
                    fourEyesRobot.intakeStop();
                }
            }
            if(binding1[2].onPress() || binding2[2].onPress()) {
                if(fourEyesRobot.isIntaking()) {
                    fourEyesRobot.intakeBackward();
                } else {
                    fourEyesRobot.intakeStop();
                }
            }

            fourEyesRobot.moveLift((controller1.left_trigger.getTriggerValue()-controller1.right_trigger.getTriggerValue())+(controller2.left_trigger.getTriggerValue()-controller2.right_trigger.getTriggerValue()));

            controller1.update();
            controller2.update();
            telemetry.update();

        }

    }
}
