package org.firstinspires.ftc.teamcode.opMode.auto;

import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.util.TeamColor;
import org.firstinspires.ftc.teamcode.util.opModes.AutoOpMode;
import org.firstinspires.ftc.teamcode.util.opModes.OpModeType;

@Autonomous(name = "Example auto", group = "auto")
public class ExampleAutoOpMode extends AutoOpMode {
    private RobotController robotController;

    @Override
    public void initialize() {
        this.robotController = new RobotController(OpModeType.AUTO, hardwareMap, telemetry, gamepad1, gamepad2, TeamColor.RED);
        this.robotController.init();
    }

    @Override
    public void sympleStart() {
        new SequentialCommandGroup(

        ).schedule();
    }

    @Override
    public void run() {
        super.run();


        telemetry.update();
    }
}
