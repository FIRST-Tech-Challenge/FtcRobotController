package org.firstinspires.ftc.teamcode.opMode.teleOp;


import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.util.TeamColor;
import org.firstinspires.ftc.teamcode.util.opModes.OpModeType;
import org.firstinspires.ftc.teamcode.util.opModes.SympleCommandOpMode;

public abstract class DriverOpMode extends SympleCommandOpMode {
    private final TeamColor teamColor;

    protected DriverOpMode(TeamColor teamColor) {
        this.teamColor = teamColor;
    }

    private RobotController robotController;

    @Override
    public void initialize() {
        this.robotController = new RobotController(OpModeType.TELE_OP, hardwareMap, telemetry, gamepad1, gamepad2, teamColor);
        this.robotController.init();
    }
}