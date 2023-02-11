package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.arm;
import org.firstinspires.ftc.teamcode.subsystems.intake;
import org.firstinspires.ftc.teamcode.subsystems.wrist;
import org.firstinspires.ftc.teamcode.teamUtil.configuredOpMode;
import org.firstinspires.ftc.teamcode.teamUtil.robotConstants;

@TeleOp(name="Configured Servo Arm Test", group="TEST")
public class configuredOpModeTest extends configuredOpMode {

    private boolean armState = true;

    private double intakePosition = 0;

    Telemetry.Item intakePositionData;

    @Override
    public void superInit() {
        r.initSystems(
                robotConstants.configuredSystems.ARM,
                robotConstants.configuredSystems.WRIST,
                robotConstants.configuredSystems.INTAKE
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
            r.intake.presetTargetPosition(intake.intakePos.OPEN);
        }
        else{
            r.intake.presetTargetPosition(intake.intakePos.CLOSED);
        }

        if(intakePosition > 1){
            intakePosition = 1;
        } else if (intakePosition < 0) {
            intakePosition = 0;
        }

        if(armState){
            r.arm.presetTargetPosition(arm.armPos.FRONT);
            r.wrist.presetTargetPosition(wrist.wristPos.FRONT);
        }
        else{
            r.arm.presetTargetPosition(arm.armPos.BACK);
            r.wrist.presetTargetPosition(wrist.wristPos.BACK);
        }

        //r.intake.freeTargetPosition(intakePosition);

        //intakePositionData.setValue(intakePosition);

        intakePositionData.setValue(r.gamepadEX1.b.isPressed());
    }

    @Override
    public void superStop() {

    }
}
