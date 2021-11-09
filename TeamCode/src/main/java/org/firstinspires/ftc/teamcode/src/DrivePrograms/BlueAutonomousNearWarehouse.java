package org.firstinspires.ftc.teamcode.src.DrivePrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.CarouselSpinner;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.Grabber;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.LinearSlide;
import org.firstinspires.ftc.teamcode.src.robotAttachments.DriveTrains.TeleopDriveTrain;

@Autonomous(name = "BlueAutonomousNearWarehouse")
public class BlueAutonomousNearWarehouse extends LinearOpMode {
    private TeleopDriveTrain driveTrain;
    private Grabber grabber;
    private LinearSlide slide;
    private CarouselSpinner spinner;

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new TeleopDriveTrain(hardwareMap, "back_left", "back_right", "front_left", "front_right");

        grabber = new Grabber(hardwareMap, "freight grabber");
        grabber.open();

        slide = new LinearSlide(hardwareMap, "linear slide arm");

        spinner = new CarouselSpinner(hardwareMap, "carousel wheel");

        telemetry.addData("Initialization Status", "Initialized");
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {
            grabber.close();
            sleep(2000);
            driveTrain.strafeAtAngle(0,0.6);
            sleep(2000);
        }

    }
}