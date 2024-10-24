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
                controller1.x,
                controller1.dpad_up,
                controller1.dpad_down

        };
        binding2 = new GamepadEvents.GamepadButton[] {
                controller2.y,
                controller2.b,
                controller2.a,
                controller2.x
        };


        waitForStart();
        fourEyesRobot.initializePowerStates();
        while (opModeIsActive()) {

            fourEyesRobot.drive(controller1.left_stick_y, controller1.left_stick_x, controller1.right_stick_x);

            //User 1 controls, driver automations
            //Left bumper
            if(binding1[0].onPress()) {
               fourEyesRobot.toggleIntake();
            }
            //Right bumper
            if(binding1[1].onPress()) {
                fourEyesRobot.toggleDeposit();
            }
            //Y button
            if(binding1[2].onPress()) {
                fourEyesRobot.intakeSamplePos();
            }
            //B button
            if(binding1[3].onPress()) {
                fourEyesRobot.intakeSpeciminPos();
            }
            //A button
            if(binding1[4].onPress()) {
                fourEyesRobot.depositSpeciminPos();
            }
            //X button
            if(binding1[5].onPress()) {
                fourEyesRobot.depositSamplePos();
            }
            //Dpad Up
            if(binding1[6].onPress()){
                fourEyesRobot.raiseClimb();
            }
            //Dpad Down
            if(binding1[7].onPress()){
                fourEyesRobot.lowerClimb();
            }

            //Player 2 controls
            //Manual driving
            //Lift manual control
            fourEyesRobot.moveLift(-controller2.left_stick_y);
            //Arm manual control
            fourEyesRobot.changeHeightArm(controller2.right_stick_y);
            //Wrist manual control
            fourEyesRobot.setWristPosition(controller2.right_trigger.getTriggerValue() - controller2.left_trigger.getTriggerValue());

            //Active intake manual control
            if(binding2[0].onPress()){
                if (fourEyesRobot.isIntaking()){
                    fourEyesRobot.deactivateIntake();
                }
                else {
                    fourEyesRobot.intakeBackward();
                }
            }
            if(binding2[3].onPress()){
                if (fourEyesRobot.isIntaking()){
                    fourEyesRobot.deactivateIntake();
                }
                else{
                    fourEyesRobot.activateIntake();
                }
            }
            //Claw manual control
            if(binding2[1].onPress()){
                fourEyesRobot.openClaw();
            }
            if(binding2[2].onPress()){
                fourEyesRobot.closeClaw();
            }


            fourEyesRobot.updatePID();
            controller1.update();
            controller2.update();
            telemetry.addLine(fourEyesRobot.toString());
            telemetry.update();

        }

    }
}
