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

        };
        binding2 = new GamepadEvents.GamepadButton[] {

        };


        waitForStart();
        while (opModeIsActive()) {
            if(binding1[0].onPress()) {
                fourEyesRobot.openClaw();
            }
            if(binding1[1].onPress()) {
                fourEyesRobot.closeClaw();

            }
            if(binding1[2].onPress()) {
                if(fourEyesRobot.isIntakeing()) {
                    fourEyesRobot.intakeForward();
                } else {
                    fourEyesRobot.intakeStop();
                }
            }
            if(binding1[3].onPress()) {
                if(fourEyesRobot.isIntakeing()) {
                    fourEyesRobot.intakeBackward();
                } else {
                    fourEyesRobot.intakeStop();
                }
            }


        }

    }
}
