package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FourEyesRobot;
import org.firstinspires.ftc.teamcode.utils.GamepadEvents;

@TeleOp(name="Pepto Bismal")
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
                controller1.left_bumper,
                controller1.right_bumper,
                controller1.y,
                controller1.b,
                controller1.a,
                controller1.x
        };
        binding2 = new GamepadEvents.GamepadButton[] {
                controller2.left_bumper,
                controller2.right_bumper,
                controller2.y,
                controller2.b,
                controller2.a,
                controller2.x
        };


        waitForStart();
        fourEyesRobot.initializePowerStates();
        while (opModeIsActive()) {

            fourEyesRobot.drive(controller1.left_stick_y, controller1.left_stick_x, controller1.right_stick_x);

            //Left bumper
            if(binding1[0].onPress() || binding2[0].onPress()) {
               fourEyesRobot.toggleIntake();
            }
            //Right bumper
            if(binding1[1].onPress() || binding2[1].onPress()) {
                fourEyesRobot.toggleDeposit();
            }
            //Y button
            if(binding1[2].onPress() || binding2[2].onPress()) {
                fourEyesRobot.intakeSamplePos();
            }
            //B button
            if(binding1[3].onPress() || binding2[3].onPress()) {
                fourEyesRobot.intakeSpeciminPos();
            }
            //A button
            if(binding1[4].onPress() || binding2[4].onPress()) {
                fourEyesRobot.depositSpeciminPos();
            }
            //X button
            if(binding1[5].onPress() || binding2[5].onPress()) {
                fourEyesRobot.depositSamplePos();
            }

            fourEyesRobot.updatePID();
            controller1.update();
            controller2.update();
            telemetry.addLine(fourEyesRobot.toString());
            telemetry.update();

        }

    }
}
