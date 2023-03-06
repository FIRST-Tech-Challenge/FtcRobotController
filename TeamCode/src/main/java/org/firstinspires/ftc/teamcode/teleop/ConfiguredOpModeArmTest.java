package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.Trigger;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.gamepadEX.ButtonEX;
import org.firstinspires.ftc.teamcode.teamUtil.ConfiguredOpMode;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConstants;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.gamepadEX.GamepadEX;

@TeleOp(name="Configured Servo Arm Test", group="TEST")
public class ConfiguredOpModeArmTest extends ConfiguredOpMode {

    private Arm arm;
    private Wrist wrist;
    private Intake intake;
    private LimitSwitch limitSwitch;
    private Lift lift;

    boolean armstate = true;

    @Override
    public void superInit() {
        arm = new Arm();
        wrist = new Wrist();
        intake = new Intake();
        limitSwitch = new LimitSwitch();
        lift = new Lift();
    }

    @Override
    public void registerTriggers() {
        /*
        Trigger intakeTrigger = new Trigger(gamepadEX2.right_bumper.isPressed())
                .onTrue(() -> intake.presetTargetPosition(Intake.intakePos.OPEN))
                .onFalse(() -> intake.presetTargetPosition(Intake.intakePos.CLOSED));
        Trigger armWristTrigger = new Trigger(gamepadEX2.left_bumper.onPress())
                .toggleOnTrue(() -> {
                    arm.presetTargetPosition(Arm.armPos.FRONT);
                    wrist.presetTargetPosition(Wrist.wristPos.FRONT);
                })
                .toggleOnFalse(() -> {
                    arm.presetTargetPosition(Arm.armPos.BACK);
                    wrist.presetTargetPosition(Wrist.wristPos.BACK);
                });

         */

    }

    @Override
    public void superInit_Loop() {

    }

    @Override
    public void superStart() {

    }

    @Override
    public void superLoop() {
        lift.liftInputs(gamepadEX2.rightY, limitSwitch.limitSwitchEX, new ButtonEX[]{gamepadEX2.y, gamepadEX2.b, gamepadEX2.x, gamepadEX2.a});

        if(gamepadEX2.right_bumper.isPressed()){
            intake.presetTargetPosition(Intake.intakePos.CLOSED);
        }
        else{
            intake.presetTargetPosition(Intake.intakePos.OPEN);
        }
        if(gamepadEX2.left_bumper.onPress()){
            armstate = ! armstate;
        }

        if(armstate){
            arm.presetTargetPosition(Arm.armPos.FRONT);
            wrist.presetTargetPosition(Wrist.wristPos.BACK);
        }
        else{
            arm.presetTargetPosition(Arm.armPos.BACK);
            wrist.presetTargetPosition(Wrist.wristPos.FRONT);
        }


    }

    @Override
    public void superStop() {

    }
}
