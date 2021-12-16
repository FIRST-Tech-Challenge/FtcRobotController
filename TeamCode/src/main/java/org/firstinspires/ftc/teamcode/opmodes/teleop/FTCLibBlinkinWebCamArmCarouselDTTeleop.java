package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateArm;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateBC4HDrive;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateCarousel;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateIntake;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateLEDs;
import org.firstinspires.ftc.teamcode.opmodes.createmechanism.CreateWebCam;


@TeleOp(name="Telop: FTCLib BlinkinWebCamArmCarouselDT Example", group="FTCLib")
public class FTCLibBlinkinWebCamArmCarouselDTTeleop extends CommandOpMode {

    @Override
    public void initialize() {

        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();

        //set up Game pad 1
        GamepadEx driveOp = new GamepadEx(gamepad1);

        //set up Game pad 2
        GamepadEx toolOp2 = new GamepadEx(gamepad2);

        CreateArm createArm = new CreateArm(hardwareMap,"arm", toolOp2, telemetry, true);

        CreateWebCam createWebCam = new CreateWebCam(hardwareMap, "Webcam 1", dashboard,createArm.getArm(), telemetry, true);
        //create LED SubSystem
        CreateLEDs createLEDs = new CreateLEDs(hardwareMap, "blinkin", driveOp, true);
        CreateCarousel createCarousel = new CreateCarousel(hardwareMap,"carousel",toolOp2,telemetry,true);
        CreateIntake createIntake = new CreateIntake(hardwareMap, "intake", toolOp2, telemetry, true);
        CreateBC4HDrive createBC4HDrive = new CreateBC4HDrive(hardwareMap,"frontLeft", "backLeft", "frontRight", "backRight", driveOp, telemetry, true );

        CommandScheduler.getInstance().run();


    }
}
