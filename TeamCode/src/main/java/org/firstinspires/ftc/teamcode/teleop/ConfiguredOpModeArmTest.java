package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.*;
import org.firstinspires.ftc.teamcode.teamUtil.ConfiguredOpMode;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConstants;
import org.firstinspires.ftc.teamcode.teamUtil.gamepadEX.CommandController;
import org.firstinspires.ftc.teamcode.teamUtil.gamepadEX.GamepadEX;

@TeleOp(name="Configured Servo Arm Test", group="TEST")
public class ConfiguredOpModeArmTest extends ConfiguredOpMode {

    private boolean armState = true;

    private double intakePosition = 0;

    Telemetry.Item intakePositionData;

    private Arm arm;
    private Wrist wrist;
    private Intake intake;
    private GamepadEX gamepadEX1;
    private GamepadEX gamepadEX2;
    CommandController commandController;
    @Override
    public void superInit() {
        r.initSystems(
            RobotConstants.configuredSystems.ARM,
            RobotConstants.configuredSystems.WRIST,
            RobotConstants.configuredSystems.INTAKE,
            RobotConstants.configuredSystems.LIFT
        );
        arm = r.getSubsystem(RobotConstants.configuredSystems.ARM);
        wrist = r.getSubsystem(RobotConstants.configuredSystems.WRIST);
        intake = r.getSubsystem(RobotConstants.configuredSystems.INTAKE);
    }

    @Override
    public void superInit_Loop() {

    }

    @Override
    public void superStart() {
        intakePositionData = telemetry.addData("intake position", "");
    }

    @Override
    public void superLoop() {
        commandController.conditionalAction(() -> gamepadEX1.a.onPress(), () -> armState = !armState);

        /*
        r.commandControl.conditionalAction(() -> r.gamepadEX1.dpad_up.onPress(), () -> intakePosition += 0.1);
        r.commandControl.conditionalAction(() -> r.gamepadEX1.dpad_down.onPress(), () -> intakePosition -= 0.1);
         */

        if(gamepadEX1.b.isPressed()){
            intake.presetTargetPosition(Intake.intakePos.OPEN);
        }
        else{
            intake.presetTargetPosition(Intake.intakePos.CLOSED);
        }

        if(intakePosition > 1){
            intakePosition = 1;
        } else if (intakePosition < 0) {
            intakePosition = 0;
        }

        if(armState){
            arm.presetTargetPosition(Arm.armPos.FRONT);
            wrist.presetTargetPosition(Wrist.wristPos.FRONT);
        }
        else{
            arm.presetTargetPosition(Arm.armPos.BACK);
            wrist.presetTargetPosition(Wrist.wristPos.BACK);
        }

        //intake.freeTargetPosition(intakePosition);

        //intakePositionData.setValue(intakePosition);

        intakePositionData.setValue(gamepadEX1.b.isPressed());
    }

    @Override
    public void superStop() {

    }
}
