package org.firstinspires.ftc.teamcode.src.v2.TeleOp;

//Import EVERYTHING we need
//import com.acmerobotics.dashboard.config.Config;
//import com.outoftheboxrobotics.photoncore.PhotonCore;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.src.v2.Subsystem.SwerveDrive;
import org.firstinspires.ftc.teamcode.src.v2.maths.PIDcontroller;


//chub: 0 - deposit 1, 2 - deposit 2, 3 - claw, 4 - turret
//exhub: 0 - intake 1, 2 - intake 2, 4 - linkage, 5 - aligner

@TeleOp(name = "Swerve")
public class Swerve extends LinearOpMode {

    enum cycleStates {
        INTAKE_GRAB,
        INTAKE_UP,
        DEPOSIT_EXTEND,
        DEPOSIT_DUMP,
        MANUAL,
        TRANSFER,
        DRIVE
    }

    cycleStates cyclestate = cycleStates.MANUAL;

    ElapsedTime timer = new ElapsedTime();

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        //Bulk sensor reads
        LynxModule controlHub = hardwareMap.get(LynxModule.class, "Control Hub");

        //class to swerve the swerve
        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap, true);

        ElapsedTime hztimer = new ElapsedTime();

        //class that runs our linear slide

//        Deposit deposit = new Deposit(hardwareMap);
//        Intake intake   = new Intake(hardwareMap);
//        Turret turret   = new Turret(hardwareMap);
//        Linkage linkage = new Linkage(hardwareMap);
//        Claw claw       = new Claw(hardwareMap);
//
//        ButtonDetector right_trigger = new ButtonDetector();
//        ButtonDetector right_bumper  = new ButtonDetector();
//        ButtonDetector left_bumper   = new ButtonDetector();
//        ButtonDetector dpad_left     = new ButtonDetector();
//        ButtonDetector button_b      = new ButtonDetector();
//        ButtonDetector button_a      = new ButtonDetector();

        PIDcontroller headingPID = new PIDcontroller(6, 0, 5, 0, 0.1);
        double headingOut, headingTarget = 0;

        //Bulk sensor reads
        controlHub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);

        //Fast loop go brrr
//        PhotonCore.enable();
//        PhotonCore.experimental.setMaximumParallelCommands(8);
//
//        slide.addOffset(right.slider);

        //linkage.setPosition(init);
        //claw.setPosition(0.6);
        //outRotL.setPosition(1);
        //outRotR.setPosition(1-outRotL.getPosition());
        //inRotL.setPosition(0.3);
        //inRotR.setPosition(1-inRotL.getPosition());

        waitForStart();
        while (opModeIsActive()) {

            //Clear the cache for better loop times (bulk sensor reads)
            controlHub.clearBulkCache();

            if (opModeIsActive()) {
                headingOut = headingPID.pidOut(AngleUnit.normalizeRadians(headingTarget - swerve.getHeading() * (Math.PI / 180)));
            } else {
                headingOut = 0;
            }

            swerve.drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y, headingOut + gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x);


            if (gamepad1.a) {
                swerve.resetIMU();
            }

        }
    }
}