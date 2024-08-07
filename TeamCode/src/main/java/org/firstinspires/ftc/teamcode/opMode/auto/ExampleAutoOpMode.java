package org.firstinspires.ftc.teamcode.opMode.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.opMode.templates.AutoOpModeTemplate;
import org.firstinspires.ftc.teamcode.util.DataLogger;
import org.firstinspires.ftc.teamcode.util.TeamColor;
import org.firstinspires.ftc.teamcode.util.opModes.AutoOpMode;

@Autonomous(name = "Example auto", group = "auto")
public class ExampleAutoOpMode extends AutoOpMode {
    private RobotController robotController;

    @Override
    public void initialize() {
        this.robotController = new RobotController(AutoOpModeTemplate.class, hardwareMap, telemetry, gamepad1, gamepad2, TeamColor.RED, true);
        this.robotController.initSubSystems();
    }

    @Override
    public void sympleStart() {
        this.robotController.getDataLogger().addData(DataLogger.DataType.INFO, this.getClass().getSimpleName() + ": Started to execute the robot path");
        new SequentialCommandGroup(

        ).schedule();
    }

    @Override
    public void run() {
        super.run();
        this.robotController.run();

        telemetry.update();
    }
}
