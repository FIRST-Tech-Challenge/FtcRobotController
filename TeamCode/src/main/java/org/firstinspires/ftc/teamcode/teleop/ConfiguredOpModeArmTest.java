package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.teamUtil.CommandScheduler.ConfiguredOpMode;

@TeleOp(name="Configured Servo Arm Test", group="TEST")
public class ConfiguredOpModeArmTest extends ConfiguredOpMode {

    private Arm arm;
    private Wrist wrist;
    private Intake intake;
    private LimitSwitch limitSwitch;
    private Lift lift;
    
    private boolean armstate;

    @Override
    public void superInit() {
        arm = new Arm();
        wrist = new Wrist();
        intake = new Intake();
        limitSwitch = new LimitSwitch();
        lift = new Lift();
        telemetry.setAutoClear(true);
    }

    @Override
    public void registerTriggers() {
        gamepadEX2.rightY.applyDeadZone(0.2);
        
        gamepadEX1.right_bumper
                .isPressed()
                    .onTrue(() -> intake.presetTargetPosition(Intake.IntakePos.OPEN))
                    .onFalse(() -> intake.presetTargetPosition(Intake.IntakePos.CLOSED));
    
    
        gamepadEX1.left_bumper
                .onPress()
                    .toggleOnTrue(() -> {
                        arm.presetTargetPosition(Arm.ArmPos.BACK);
                        wrist.presetTargetPosition(Wrist.WristPos.BACK);
                    })
                    .toggleOnFalse(() -> {
                        arm.presetTargetPosition(Arm.ArmPos.FRONT);
                        wrist.presetTargetPosition(Wrist.WristPos.FRONT);
                    });
    
    }

    @Override
    public void superInit_Loop() {
    
    }

    @Override
    public void superStart() {
//        if(gamepad2.left_bumper){
//            armstate = !armstate;
//        }
//        if(armstate){
//            arm.presetTargetPosition(Arm.armPos.FRONT);
//            wrist.presetTargetPosition(Wrist.wristPos.BACK);
//        }
//        else{
//            arm.presetTargetPosition(Arm.armPos.BACK);
//            wrist.presetTargetPosition(Wrist.wristPos.FRONT);
//        }
//
//        if(gamepad1.right_bumper) {
//            intake.presetTargetPosition(Intake.intakePos.OPEN);
//        }
//        else{
//            intake.presetTargetPosition(Intake.intakePos.CLOSED);
//        }
    }

    @Override
    public void superLoop() {
        lift.liftInputs(gamepadEX2.rightY.getValue(), limitSwitch.limitSwitchEX.buttonState(), gamepadEX2.y.buttonState(), gamepadEX2.b.buttonState(), gamepadEX2.x.buttonState(), gamepadEX2.a.buttonState());
    }

    @Override
    public void superStop() {

    }
}
