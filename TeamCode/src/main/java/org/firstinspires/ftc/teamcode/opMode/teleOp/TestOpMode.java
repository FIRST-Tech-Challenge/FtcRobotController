package org.firstinspires.ftc.teamcode.opMode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.util.TeamColor;
import org.firstinspires.ftc.teamcode.util.opModes.OpModeType;
import org.firstinspires.ftc.teamcode.util.opModes.SympleCommandOpMode;

@TeleOp(name = "Robot Testing", group = "test")
public class TestOpMode extends SympleCommandOpMode {
    private final TeamColor teamColor = TeamColor.RED;

    private RobotController robotController;

    @Override
    public void initialize() {
        this.robotController = new RobotController(OpModeType.DEBUG, hardwareMap, telemetry, gamepad1, gamepad2, this.teamColor);
        this.robotController.init();
    }
}
