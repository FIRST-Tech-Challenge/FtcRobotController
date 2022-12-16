package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.util.DriveForTest;
import org.firstinspires.ftc.teamcode.util.IntegratedLocalizerIMU;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.TowerController;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="ClobberHope", group="Basic drive")
public class TeleOp extends OpMode

{
    private IntegratedLocalizerIMU localizer = null;
    private MecanumDriveBase mecanumDriveBase = null;
    private TowerController towerController;
    private PacManTurnToPos pacMan;
    private DriveForTest driveForTest;

    public void init() {
        // Commented out till merge with master
        telemetry.addData("Status", "Initialized");
//        localizer = new IntegratedLocalizerIMU(hardwareMap);
//        mecanumDriveBase = new MecanumDriveBase(hardwareMap);
        towerController = new TowerController(hardwareMap, telemetry);
//        localizer = new LocalizerIMU(hardwareMap);
//        pacMan = new PacManTurnToPos(localizer, mecanumDriveBase);
//        double max;
        driveForTest = new DriveForTest(hardwareMap);
    }
    @Override
    public void loop() {
//        localizer.displayTelemetry(telemetry);
//        localizer.handleTracking();
//        mecanumDriveBase.gamepadController(gamepad1);
//        mecanumDriveBase.driveBaseTelemetry(telemetry);
        towerController.handleGamepad(gamepad2, telemetry);
//        telemetry.addData("TeleOp heading", localizer.getHeading() );
//        pacMan.handlePacMan(gamepad1, telemetry);
        driveForTest.drive(gamepad1);
        telemetry.update();
    }
}