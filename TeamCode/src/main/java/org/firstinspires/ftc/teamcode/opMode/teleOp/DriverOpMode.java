package org.firstinspires.ftc.teamcode.opMode.teleOp;


import org.firstinspires.ftc.teamcode.RobotController;
import org.firstinspires.ftc.teamcode.util.TeamColor;
import org.firstinspires.ftc.teamcode.util.opModes.OpModeType;
import org.firstinspires.ftc.teamcode.util.opModes.SympleCommandOpMode;

public abstract class DriverOpMode extends SympleCommandOpMode {
    private TeamColor teamColor;
    private RobotController robotController;

    public void initialize(TeamColor teamColor) {
        this.teamColor = teamColor;

        this.robotController = new RobotController(OpModeType.TELE_OP, hardwareMap, telemetry, gamepad1, gamepad2, teamColor);
        this.robotController.init();
    }
}