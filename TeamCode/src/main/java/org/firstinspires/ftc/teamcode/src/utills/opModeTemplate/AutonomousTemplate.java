package org.firstinspires.ftc.teamcode.src.utills.opModeTemplate;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.linearSlide.HeightLevel;

/**
 * A template for Autonomous OpModes, allows for easy initialization
 */
@Disabled
public abstract class AutonomousTemplate extends GenericOpModeTemplate {

    protected SampleMecanumDrive drive;


    /**
     * Initializes all fields provided by this class
     *
     * @throws InterruptedException Throws if the OpMode is stopped during function execution
     */
    protected void initAll() throws InterruptedException {
        drive = new SampleMecanumDrive(hardwareMap);
        super.initAll();
        podServos.lower();
        slide.setTargetLevel(HeightLevel.Down);

        telemetry.addData("Default Initialization: ", "Finished");
        telemetry.update();
        checkStop();

    }

}
