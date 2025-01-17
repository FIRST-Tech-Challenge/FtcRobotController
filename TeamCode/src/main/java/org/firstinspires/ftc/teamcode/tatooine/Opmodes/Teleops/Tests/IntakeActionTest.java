package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Camera;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Intake;
import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.EasyGamepad;
import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.GamepadKeys;

import java.util.ArrayList;
import java.util.List;
@Disabled

@TeleOp

public class IntakeActionTest extends LinearOpMode {
    private List<Action> runningActions = new ArrayList<>();
    private EasyGamepad easyGamepad1;

    private boolean dPAdRightToggle = false;

    private Arm arm;

    private Wrist wrist;

    private Camera camera;

    private Intake intake;

    private boolean lockExtend = true;

    @Override
    public void runOpMode(){telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        easyGamepad1 = new EasyGamepad(gamepad1);
        arm = new Arm(this, false);
        wrist = new Wrist(this, false);
        intake = new Intake(this,false, false);
        camera = new Camera(this, true , false);
        waitForStart();
        while (opModeIsActive()) {
            easyGamepad1.update(gamepad1);
            TelemetryPacket packet = new TelemetryPacket();

            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {

                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;
            if (easyGamepad1.justPressedButton(GamepadKeys.Button.CROSS)){
                lockExtend = !lockExtend;
            }
            else if (easyGamepad1.justPressedButton(GamepadKeys.Button.CIRCLE)){
                runningActions.clear();
                runningActions.add(wrist.moveToAngle(camera.getAngle()));
            }
            if (!lockExtend){
                arm.setPowerExtend(easyGamepad1.getStick(GamepadKeys.Stick.RIGHT_STICK_Y));
            }
            if (easyGamepad1.justPressedButton(GamepadKeys.Button.DPAD_RIGHT) && !dPAdRightToggle){
                lockExtend = true;
                dPAdRightToggle = true;
                runningActions.clear();
                wrist.setPosAng(0.5);
                wrist.stright();
                runningActions.add(arm.moveAngle());
                runningActions.add(new SequentialAction(arm.setAngle(15), new InstantAction(() ->wrist.stright()), arm.setExtension(Arm.getMaxExtend()/2), wrist.moveToAngle(60)));
            }
            else if (easyGamepad1.justPressedButton(GamepadKeys.Button.DPAD_RIGHT) && dPAdRightToggle){
                lockExtend = true;
                dPAdRightToggle = false;
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(new SequentialAction(arm.setAngle(15* ((arm.getMinExtend() +arm.getExtend()))/arm.getMinExtend()),new InstantAction(()-> wrist.openMin()),intake.intake(),new SleepAction(2),arm.setAngle(-8), new SleepAction(2), intake.setPowerAction(0)));
            } else if (easyGamepad1.justPressedButton(GamepadKeys.Button.DPAD_DOWN)) {
                lockExtend = true;
                runningActions.clear();
                runningActions.add(arm.moveAngle());
                runningActions.add(new SequentialAction(arm.setAngle(15), new InstantAction(()->wrist.stright()),new InstantAction(()-> wrist.moveToAngle(60)),  arm.setExtension(0) , new InstantAction(()-> wrist.home()), arm.setAngle(0)));
            }
            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            telemetry.update();
        }


    }
}
