package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops.StateTest;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Line;
import com.acmerobotics.roadrunner.SequentialAction;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops.ActionTeleOp;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;
import org.firstinspires.ftc.teamcode.tatooine.utils.DebugUtils;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.EasyGamepad;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.GamepadKeys;

import java.util.ArrayList;
import java.util.List;


@TeleOp
public class HomeStateTest extends LinearOpMode {
    private List<Action> runningActions = new ArrayList<>();
    private Arm arm;
    private Wrist wrist;

    private boolean doOne = true;

    private boolean doStateOne = false;

    private enum State {
        DEFULT,
        HOME,
        INTAKING1,
        INTAKING2,
        SCORE_SAMPLE,
        SCORE_SPECIMEN
    }
    private State currentState = State.DEFULT;


    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        arm = new Arm(this, true);
        wrist = new Wrist(this, true);
        EasyGamepad gamepadEx1 = new EasyGamepad(gamepad1);
        EasyGamepad gamepadEx2 = new EasyGamepad(gamepad2);

        waitForStart();
        while (opModeIsActive()){
            TelemetryPacket packet = new TelemetryPacket();
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;

            if (doOne){
                runningActions.add(arm.moveAngle());
                doOne = false;
            }

            if (gamepadEx2.justPressedButton(GamepadKeys.Button.DPAD_DOWN)){
                doStateOne = true;
                currentState = State.DEFULT;
            }
            if (doStateOne) {
                switch (currentState) {
                    case DEFULT:
                        handleDefultState();
                        break;
                }
                doStateOne = false;
            }



            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            gamepadEx1.update(gamepad1);
            gamepadEx2.update(gamepad2);
            DebugUtils.logDebug(telemetry, true, "tele", "state", currentState);
            telemetry.update();
        }
    }
    private void handleDefultState(){
        runningActions.add(new SequentialAction(arm.setAngle(0), arm.setExtension(1/20* Arm.getMaxExtend()), new InstantAction(()-> wrist.home()), wrist.moveToAngle(0)));
    }
}
