package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Arm;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Wrist;
import org.firstinspires.ftc.teamcode.teamUtil.ConfiguredOpMode;
import org.firstinspires.ftc.teamcode.teamUtil.RobotConstants;

@TeleOp(name="Configured Servo Arm Test", group="TEST")
public class ConfiguredOpModeArmTest extends ConfiguredOpMode {

    private boolean armState = true;

    private double intakePosition = 0;

    Telemetry.Item intakePositionData;

    @Override
    public void superInit() {
        r.initSystems(
                RobotConstants.configuredSystems.ARM,
                RobotConstants.configuredSystems.WRIST,
                RobotConstants.configuredSystems.INTAKE,
                RobotConstants.configuredSystems.LIFT
        );
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
        r.commandControl.conditionalAction(() -> r.gamepadEX1.a.onPress(), () -> armState = !armState);

        /*
        r.commandControl.conditionalAction(() -> r.gamepadEX1.dpad_up.onPress(), () -> intakePosition += 0.1);
        r.commandControl.conditionalAction(() -> r.gamepadEX1.dpad_down.onPress(), () -> intakePosition -= 0.1);
         */

        if(r.gamepadEX1.b.isPressed()){
            r.intake.presetTargetPosition(Intake.intakePos.OPEN);
        }
        else{
            r.intake.presetTargetPosition(Intake.intakePos.CLOSED);
        }

        if(intakePosition > 1){
            intakePosition = 1;
        } else if (intakePosition < 0) {
            intakePosition = 0;
        }

        if(armState){
            r.arm.presetTargetPosition(Arm.armPos.FRONT);
            r.wrist.presetTargetPosition(Wrist.wristPos.FRONT);
        }
        else{
            r.arm.presetTargetPosition(Arm.armPos.BACK);
            r.wrist.presetTargetPosition(Wrist.wristPos.BACK);
        }

        //r.intake.freeTargetPosition(intakePosition);

        //intakePositionData.setValue(intakePosition);

        intakePositionData.setValue(r.gamepadEX1.b.isPressed());
    }

    @Override
    public void superStop() {

    }
}
