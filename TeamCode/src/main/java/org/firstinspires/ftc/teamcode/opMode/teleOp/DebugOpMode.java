package org.firstinspires.ftc.teamcode.opMode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.opMode.templates.DebugOpModeTemplate;
import org.firstinspires.ftc.teamcode.util.TeamColor;
import org.firstinspires.ftc.teamcode.util.opModes.SympleCommandOpMode;

@TeleOp(name = "Robot Testing", group = "test")
public class DebugOpMode extends SympleCommandOpMode {
    private final TeamColor teamColor = TeamColor.RED;

    private RobotController robotController;

    @Override
    public void initialize() {
        this.robotController = new RobotController(DebugOpModeTemplate.class, hardwareMap, telemetry, gamepad1, gamepad2, this.teamColor, true);
        this.robotController.initSubSystems();
    }

    @Override
    public void run() {
        super.run();
        this.robotController.run();
    }
}
