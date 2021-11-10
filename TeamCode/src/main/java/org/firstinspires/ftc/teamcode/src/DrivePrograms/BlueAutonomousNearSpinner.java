package org.firstinspires.ftc.teamcode.src.DrivePrograms;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.src.Utills.AutonomousTemplate;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.CarouselSpinner;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.Grabber;
import org.firstinspires.ftc.teamcode.src.robotAttachments.Subsystems.LinearSlide;


@Autonomous(name = "BlueAutonomousNearSpinner")
public class BlueAutonomousNearSpinner extends AutonomousTemplate {
    private Grabber grabber;
    private LinearSlide slide;
    private CarouselSpinner spinner;

    @Override
    public void runOpMode() throws InterruptedException {
        this.initAll();
        waitForStart();
        telemetry.addData("HI", 0);
        telemetry.update();
        driveSystem.moveToPosition(12, 0, 1, true); //TODO Verify that this moves correctly

    }
}
