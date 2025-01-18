//package org.firstinspires.ftc.masters.old;
//
//
//import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.dashboard.config.Config;
//import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.Disabled;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
//import com.qualcomm.robotcore.util.ElapsedTime;
//
//import org.firstinspires.ftc.masters.old.CSCons.DriveMode;
//import org.firstinspires.ftc.masters.components.DriveTrain;
//
//import java.util.List;
//
//@Disabled
//@Config
//@TeleOp(name = "Lobster Cup Teleop", group = "competition")
//public class LobsterCupTeleop extends LinearOpMode {
//
//    DriveTrain drivetrain = null;
//    Outake outake;
//    Intake intake;
//    Transfer transfer;
//    Shooter shooter;
//
//
//    private final ElapsedTime runtime = new ElapsedTime();
//
//    private DriveMode driveMode = DriveMode.NORMAL;
//
//    @Override
//    public void runOpMode() {
//
//        drivetrain = new DriveTrain(hardwareMap);
//        transfer = Transfer.getInstance(hardwareMap, telemetry);
//
//        outake = new Outake(hardwareMap, telemetry);
//        intake = new Intake(hardwareMap, telemetry);
//        shooter = new Shooter(hardwareMap);
//
//
//        /*
//    CONTROLS:
//    TODO: Left stick - drive
//    TODO: Triggers - turning
//    DRIVE CONTROLS:
//    TODO: Right stick - intake:
//        up - intake on
//        down - intake reverse
//        right - intake off
//    TODO: D-pad - stack control
//        up - intake top // Done
//        down - intake bottom // Done
//        left - intake down by 1
//        right - stack up by 1
//    TODO: Touchpad:
//        two finger click - drone
//    TODO: stick Buttons:
//        left stick - release transfer
//        right stick - transfer
//    SCORE CONTROLS:
//    TODO: ABXY & Back Bumper - outtake control initial
//        A - outtake vertical
//        Y - outtake vertical flip
//        X - outtake horizontal
//        B - outtake horizontal flip
//        LB - outtake mid
//        RB - outtake high
//        NB - outtake low
//    TODO: ABXY & Back Bumper - outtake control final
//        vertical:
//            A - bottom pixel drop
//            Y - top pixel drop
//        horizontal:
//            X - left pixel drop
//            B - right pixel drop
//        angled right:
//            X - left pixel drop
//            Y - right pixel drop
//        angled left:
//            Y - left pixel drop
//            B - right pixel drop
//        right stick:
//            up - grab
//            down - drop
//            right - drive mode
//            left - retract
//
//        dpad:
//            up - vertical
//            down - vertical flip
//            left - rotate left
//            right - rotate right
//       touchpad:
//            left - flat
//            right - flat flip
//                outtake up down:
//            share - down
//            options - up
//    HANG CONTROLS:
//    TODO: PS button - hang mode
//        sets up hang
//        A - hang
//        */
//
//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }
//
//
//        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
//
//
//        waitForStart();
//
//        runtime.reset();
//        ElapsedTime elapsedTime;
//
//        // run until the end of the match (driver presses STOP)
//        while (opModeIsActive()) {
//            telemetry.addData("DRIVE MODE", driveMode.name());
//
//
//            if (gamepad1.share) {
//                driveMode = DriveMode.HANG;
//            }
//            if (driveMode == DriveMode.HANG){
//                if (gamepad1.right_stick_x>0.2){
//                    driveMode = DriveMode.NORMAL;
//                }
//            }
//
//            drivetrain.drive(gamepad1);
//            transfer.update(gamepad1, outake.getOuttakeState());
//            intake.update(gamepad1, driveMode, outake.getOuttakeState());
//            outake.update(gamepad1, driveMode);
//            shooter.update(gamepad1);
//
//            telemetry.update();
//        }
//    }
//
//
//
//
//}
