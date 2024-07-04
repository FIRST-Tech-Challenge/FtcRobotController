package org.firstinspires.ftc.masters;


import static org.firstinspires.ftc.masters.CSCons.servo1Down;
import static org.firstinspires.ftc.masters.CSCons.servo1Up;
import static org.firstinspires.ftc.masters.CSCons.servo2Down;
import static org.firstinspires.ftc.masters.CSCons.servo2Up;
import static org.firstinspires.ftc.masters.CSCons.transferPush;
import static org.firstinspires.ftc.masters.CSCons.transferUp;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.masters.CSCons.DriveMode;
import org.firstinspires.ftc.masters.CSCons.HookPosition;
import org.firstinspires.ftc.masters.CSCons.OuttakePosition;
import org.firstinspires.ftc.masters.CSCons.OuttakeState;
import org.firstinspires.ftc.masters.CSCons.OuttakeWrist;
import org.firstinspires.ftc.masters.components.DriveTrain;
import org.firstinspires.ftc.masters.components.Intake;
import org.firstinspires.ftc.masters.components.Outake;
import org.firstinspires.ftc.masters.components.Shooter;
import org.firstinspires.ftc.masters.components.Transfer;

import java.util.List;
import java.util.concurrent.TimeUnit;


@Config
@TeleOp(name = "Lobster Cup Teleop", group = "competition")
public class LobsterCupTeleop extends LinearOpMode {

    DriveTrain drivetrain = null;
    Outake outake;
    Intake intake;
    Transfer transfer;
    Shooter shooter;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    private final ElapsedTime runtime = new ElapsedTime();


//    DcMotor intake = null;
//    Servo intakeHeight = null;

    Servo planeRaise;

    Servo outtakeRotation;
    Servo outtakeMovement;
    Servo outtakeMovementRight;

//    Servo transferServo;

    private Servo wristServo;
    private Servo outtakeServo1, outtakeServo2;
    private OuttakeWrist outtakeWristPosition;



    int numberOfPixelsInRamp= 0;

    double outtakeRotationTarget;

    CSCons.TransferStatus currentTransferStatus;


    ElapsedTime liftFingers = new ElapsedTime();


//    private enum OuttakeWrist{
//        flatRight (CSCons.wristFlatRight), angleRight(CSCons.wristAngleRight), verticalDown(CSCons.wristVerticalDown),
//        vertical(CSCons.wristVertical), angleLeft(CSCons.wristAngleLeft), flatLeft(CSCons.wristFlatLeft);
//
//        double position;
//        private OuttakeWrist(double position){
//            this.position= position;
//        }
//
//        public double getPosition(){
//            return position;
//        }
//    }

    private DriveMode driveMode = DriveMode.NORMAL;
    private OuttakeState outtakeState = OuttakeState.ReadyToTransfer;

    private CSCons.IntakeDirection intakeDirection = CSCons.IntakeDirection.OFF;
    private HookPosition hookPosition = HookPosition.OPEN;

    @Override
    public void runOpMode() {

        drivetrain = new DriveTrain(hardwareMap);
        transfer = Transfer.getInstance(hardwareMap);

        outake = new Outake(hardwareMap);
        intake = new Intake(hardwareMap);
        shooter = new Shooter(hardwareMap);


        /*
    CONTROLS:
    TODO: Left stick - drive
    TODO: Triggers - turning
    DRIVE CONTROLS:
    TODO: Right stick - intake:
        up - intake on
        down - intake reverse
        right - intake off
    TODO: D-pad - stack control
        up - intake top // Done
        down - intake bottom // Done
        left - intake down by 1
        right - stack up by 1
    TODO: Touchpad:
        two finger click - drone
    TODO: stick Buttons:
        left stick - release transfer
        right stick - transfer
    SCORE CONTROLS:
    TODO: ABXY & Back Bumper - outtake control initial
        A - outtake vertical
        Y - outtake vertical flip
        X - outtake horizontal
        B - outtake horizontal flip
        LB - outtake mid
        RB - outtake high
        NB - outtake low
    TODO: ABXY & Back Bumper - outtake control final
        vertical:
            A - bottom pixel drop
            Y - top pixel drop
        horizontal:
            X - left pixel drop
            B - right pixel drop
        angled right:
            X - left pixel drop
            Y - right pixel drop
        angled left:
            Y - left pixel drop
            B - right pixel drop
        right stick:
            up - grab
            down - drop
            right - drive mode
            left - retract

        dpad:
            up - vertical
            down - vertical flip
            left - rotate left
            right - rotate right
       touchpad:
            left - flat
            right - flat flip
                outtake up down:
            share - down
            options - up
    HANG CONTROLS:
    TODO: PS button - hang mode
        sets up hang
        A - hang
        */


        planeRaise = hardwareMap.servo.get("planeRaise");
        wristServo = hardwareMap.servo.get("wrist");


        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        outtakeMovement.setPosition(CSCons.wristOuttakeMovementTransfer);
        outtakeMovementRight.setPosition(CSCons.wristOuttakeMovementTransfer);
        outtakeRotation.setPosition(CSCons.wristOuttakeAngleTransfer);

        hookPosition = HookPosition.OPEN;
        planeRaise.setPosition(CSCons.droneFlat);

        outtakeServo1.setPosition(CSCons.servo1Up);
        outtakeServo2.setPosition(CSCons.servo2Up);


        outtakeWristPosition = OuttakeWrist.vertical;
        wristServo.setPosition(CSCons.wristVertical);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());


        waitForStart();

        runtime.reset();
        ElapsedTime elapsedTime;

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            telemetry.addData("DRIVE MODE", driveMode.name());


            if (gamepad1.ps) {
                driveMode = DriveMode.HANG;
            }

            drivetrain.drive(gamepad1);
            transfer.update(gamepad1, outake.getOuttakeState());
            intake.update(gamepad1, driveMode, outake.getOuttakeState());
            outake.update(gamepad1, driveMode);
            shooter.update(gamepad1);


        }
    }




}
