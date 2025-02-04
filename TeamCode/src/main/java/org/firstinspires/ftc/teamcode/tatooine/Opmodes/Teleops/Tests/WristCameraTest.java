//package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops.Tests;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//
//import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Camera;
//import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Wrist;
//import org.firstinspires.ftc.teamcode.tatooine.utils.Alliance.CheckAlliance;
//import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.EasyGamepad;
//import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.GamepadKeys;
//import org.firstinspires.ftc.teamcode.tatooine.utils.mathUtil.MathUtil;
//
//import java.util.ArrayList;
//import java.util.List;
//@Disabled
//@TeleOp(name = "WristCameraTest", group = "Tests")
//public class WristCameraTest extends LinearOpMode {
//
//    private List<Action> runningActions = new ArrayList<>();
//
//    private boolean isRed = CheckAlliance.isRed();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//        EasyGamepad gamepadEx1 = new EasyGamepad(gamepad1);
//        Wrist wrist = new Wrist(this, true);
//        Camera camera = new Camera(this, true, isRed);
//        waitForStart();
//        while (opModeIsActive()){
//            TelemetryPacket packet = new TelemetryPacket();
//            List<Action> newActions = new ArrayList<>();
//            for (Action action : runningActions) {
//
//                action.preview(packet.fieldOverlay());
//                if (action.run(packet)) {
//                    newActions.add(action);
//                }
//            }
//            runningActions = newActions;
//
//            if (gamepadEx1.justPressedButton(GamepadKeys.Button.DPAD_UP)){
//                wrist.setPositionWristAngle(camera.getAngle());
//            } else if (gamepadEx1.justPressedButton(GamepadKeys.Button.START)) {
//                camera.setSpecimen(!camera.isSpecimen());
//            }
//
//
//            FtcDashboard.getInstance().sendTelemetryPacket(packet);
//
//            telemetry.update();
//
//            gamepadEx1.update(gamepad1);
//        }
//    }
//}
