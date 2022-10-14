package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.LocalizerIMU;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOpStub", group="Basic drive")
public class TeleOp extends OpMode

{
    private LocalizerIMU localizer = null;
    private MecanumDriveBase mecanumDriveBase = null;
    private PacManTurnToPos pacMan;

    public void init() {
        telemetry.addData("Status", "Initialized");
        mecanumDriveBase = new MecanumDriveBase(hardwareMap);
        localizer = new LocalizerIMU(hardwareMap);
        pacMan = new PacManTurnToPos(localizer, mecanumDriveBase);
        double max;
    }

    @Override
    public void loop() {
        localizer.displayTelemetry(telemetry);
        localizer.handleTracking();
        mecanumDriveBase.gamepadController(gamepad1);
        mecanumDriveBase.driveBaseTelemetry(telemetry);
        telemetry.addData("TeleOp heading", localizer.heading);
        pacMan.handlePacMan(gamepad1, telemetry);
        telemetry.update();

    }
}