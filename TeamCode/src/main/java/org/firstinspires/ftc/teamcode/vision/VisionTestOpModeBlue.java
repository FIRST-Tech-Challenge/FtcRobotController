package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.vision.pipeline.CapstoneDetectorBlue;

@Autonomous(name = "VisionBlueTesting")
public class VisionTestOpModeBlue extends CommandOpMode {

    private CapstoneDetectorBlue capstoneDetector;

    @Override
    public void initialize() {

        capstoneDetector = new CapstoneDetectorBlue(hardwareMap, "coolio");
        capstoneDetector.init();

        FtcDashboard.getInstance().startCameraStream(capstoneDetector.getCamera(), 30);

        schedule(new WaitCommand(5000).andThen(new RunCommand(() -> {
            telemetry.addData("Capstone Placement", capstoneDetector.getPlacement());
            telemetry.update();
        })));
      }
}