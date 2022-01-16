package org.firstinspires.ftc.teamcode.src.drivePrograms.autonomous.testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.src.robotAttachments.driveTrains.TeleopDriveTrain;
import org.firstinspires.ftc.teamcode.src.robotAttachments.sensors.RobotVoltageSensor;
import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.LinearSlide;

@Disabled
@Autonomous(name = "Linear Slide Auto Test")
public class LinearSlideAutoTest extends LinearOpMode {

    TeleopDriveTrain driveTrain;
    LinearSlide slide;

    @Override
    public void runOpMode() throws InterruptedException {
        driveTrain = new TeleopDriveTrain(hardwareMap, "front_right/vr", "front_left/vl", "back_right/h", "back_left");


        RobotVoltageSensor s = new RobotVoltageSensor(hardwareMap);
        slide = new LinearSlide(hardwareMap, "linear_slide", s, this::opModeIsActive, this::isStopRequested);
        slide.start();


        telemetry.addData("Initialization Status", "Initialized");
        telemetry.update();

        waitForStart();
        int pos = 0;
        while (opModeIsActive() && !isStopRequested()) {

            telemetry.addData("Current Pos", slide.getEncoderCount());
            telemetry.addData("Destination", slide.getTargetHeight());
            telemetry.update();

            if (slide.isAtPosition()) {
                switch (pos) {
                    case 0:
                        slide.setTargetLevel(LinearSlide.HeightLevel.Down);
                        pos++;
                        break;
                    case 1:
                        slide.setTargetLevel(LinearSlide.HeightLevel.BottomLevel);
                        pos++;
                        break;
                    case 2:
                        slide.setTargetLevel(LinearSlide.HeightLevel.MiddleLevel);
                        pos++;
                        break;

                    case 3:
                        pos++;
                        slide.setTargetLevel(LinearSlide.HeightLevel.TopLevel);
                        break;
                    case 4:
                        pos = 0;
                }
            }
        }
    }
}
