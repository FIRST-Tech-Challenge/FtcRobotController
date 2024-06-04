package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.TeamColor;
import org.firstinspires.ftc.teamcode.util.opModes.OpModeType;

public class RobotController {
    public final GamepadEx driverController;
    public final GamepadEx actionController;

    private final OpModeType opModeType;
    private final HardwareMap hardwareMap;
    private final MultipleTelemetry telemetry;
    private final TeamColor teamColor;

    public RobotController(OpModeType opModeType, HardwareMap hMap, Telemetry telemetry, Gamepad driverController, Gamepad actionController, TeamColor teamColor) {
        this.opModeType = opModeType;
        this.hardwareMap = hMap;
        this.teamColor = teamColor;
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        this.driverController = new GamepadEx(driverController);
        this.actionController = new GamepadEx(actionController);

        FtcDashboard.getInstance().stopCameraStream();
    }

    public void init() {
        CommandScheduler.getInstance().reset();

        switch (this.opModeType) {
            case TELE_OP: {
                initTeleOpMode();
                break;
            }
            case AUTO: {
                initAutoOpMode();
                break;
            }
            case DEBUG: {
                initTestingOpMode();
                break;
            }
        }
    }

    public void initTeleOpMode() {
        initDriverButtons();
        initActionButtons();
    }

    public void initAutoOpMode() {
    }

    public void initTestingOpMode() {
        initTestingButtons();
    }

    public void initDriverButtons() {
    }

    public void initActionButtons() {
    }

    public void initTestingButtons() {
    }

    public TeamColor getTeamColor() {
        return teamColor;
    }

    public MultipleTelemetry getTelemetry() {
        return telemetry;
    }
}
