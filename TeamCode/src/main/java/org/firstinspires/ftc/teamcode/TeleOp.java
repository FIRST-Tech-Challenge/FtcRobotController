package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.LocalizerIMU;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOpStub", group="Basic drive")
public class TeleOp extends OpMode

{
    private Localizer localizer = null;
    private MecanumDriveBase mecanumDriveBase = null;
    private PacManTurnToPos pacMan;

    public void init() {
        telemetry.addData("Status", "Initialized");
        localizer = new LocalizerIMU(hardwareMap);
        mecanumDriveBase = new MecanumDriveBase(hardwareMap);
        pacMan = new PacManTurnToPos(localizer, mecanumDriveBase);
//        IMUlocalizer.gyroCalibrate();
        double max;
    }
    @Override
    public void loop() {
        localizer.displayTelemetry(telemetry);
        localizer.handleTracking();
        mecanumDriveBase.gamepadController(gamepad1);
        mecanumDriveBase.driveBaseTelemetry(telemetry);
//        pacMan.handlePacMan(gamepad1, telemetry);
        telemetry.update();

    }
}
