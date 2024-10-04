//package org.firstinspires.ftc.teamcode.tatooine.Opmodes.Teleops;
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
//import com.acmerobotics.roadrunner.Action;
//import com.acmerobotics.roadrunner.Actions;
//import com.acmerobotics.roadrunner.InstantAction;
//import com.acmerobotics.roadrunner.Pose2d;
//import com.acmerobotics.roadrunner.SequentialAction;
//import com.acmerobotics.roadrunner.Vector2d;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.RoadRunner.MecanumDrive;
//import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Arm;
//import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Intake;
//import org.firstinspires.ftc.teamcode.tatooine.SubSystem.Lift;
//import org.firstinspires.ftc.teamcode.tatooine.utils.gamepads.EasyGamepad;
//
//import java.util.ArrayList;
//import java.util.List;
//
//public class Test extends LinearOpMode {
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        FtcDashboard dash = FtcDashboard.getInstance();
//        List<Action> runningActions = new ArrayList<>();
//
//        Lift lift = new Lift(this, false);
//        Arm arm = new Arm(this, false);
//        Intake intake = new Intake(hardwareMap);
//
//        EasyGamepad easyGamepad1 = new EasyGamepad(gamepad1);
//        EasyGamepad easyGamepad2 = new EasyGamepad(gamepad2);
//
//        TelemetryPacket packet = new TelemetryPacket();
//
//        MecanumDrive drive = new MecanumDrive(hardwareMap, new Pose2d(new Vector2d(0,0),0));
//
//        double extend1 = 0;
//        double extend2 = 0;
//        double extend3 = 0;
//        double extend4 = 0;
//
//        double angle1 = 0;
//        double angle2 = 0;
//        double angle3 = 0;
//
//        while (opModeIsActive()){
//            easyGamepad1.update(gamepad1);
//            easyGamepad2.update(gamepad2);
//
//            List<Action> newActions = new ArrayList<>();
//            for (Action action : runningActions) {
//                action.preview(packet.fieldOverlay());
//                if (action.run(packet)) {
//                    newActions.add(action);
//                }
//            }
//            runningActions = newActions;
//            dash.sendTelemetryPacket(packet);
//
//            drive.fieldDrive(new Pose2d(new Vector2d(easyGamepad1.getLeftStickXPower(),easyGamepad2.getLeftStickYPower()),easyGamepad1.getR2()-easyGamepad1.getL2()));
//            if (easyGamepad2.getR1()) {
//                runningActions.add(
//                intake.intake(easyGamepad2.getR1())
//                );
//            }
//
//            if(easyGamepad2.getDPadUp()){
//                runningActions.add(
//                arm.setAngle(angle1)
//                );
//            }
//
//            if(easyGamepad2.getDPadRight()){
//                runningActions.add(
//                arm.setAngle(angle2)
//                );
//            }
//
//            if(easyGamepad2.getDPadDown()){
//                runningActions.add(
//                arm.setAngle(angle3)
//                );
//            }
//
//            if(easyGamepad2.getCross()){
//                runningActions.add(
//                arm.setExtension(extend1)
//                );
//            }
//
//            if(easyGamepad2.getCircle()){
//                runningActions.add(
//                arm.setExtension(extend2)
//                );
//            }
//
//            if(easyGamepad2.getTriangle()) {
//                runningActions.add(
//                arm.setExtension(extend3)
//                );
//            }
//
//            if(easyGamepad2.getSquare()){
//                runningActions.add(
//                arm.setExtension(extend4)
//                );
//            }
//
//            if(easyGamepad2.getDPadLeft()){
//                runningActions.add(
//                lift.hanging()
//                );
//            }
//
//        }
//    }
//        }
//
