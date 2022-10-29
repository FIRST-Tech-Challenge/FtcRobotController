package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.util.IntegratedLocalizerIMU;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;
import org.firstinspires.ftc.teamcode.util.TowerController;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOpStub", group="Basic drive")
public class TeleOp extends OpMode

{
    private IntegratedLocalizerIMU localizer        = null;
    private MecanumDriveBase mecanumDriveBase = null;
    private TowerController towerController;
    private PacManTurnToPos pacMan;

    public void init() {
        telemetry.addData("Status", "Initialized");
        localizer = new IntegratedLocalizerIMU(hardwareMap);
        mecanumDriveBase = new MecanumDriveBase(hardwareMap);
        towerController = new TowerController(hardwareMap);
//        localizer = new LocalizerIMU(hardwareMap);
        pacMan = new PacManTurnToPos(localizer, mecanumDriveBase);
        double max;
    }
    @Override
    public void loop() {
        localizer.displayTelemetry(telemetry);
        localizer.handleTracking();
        mecanumDriveBase.gamepadController(gamepad1);
        mecanumDriveBase.driveBaseTelemetry(telemetry);
        telemetry.addData("TeleOp heading", localizer.getHeading() );
        pacMan.handlePacMan(gamepad1, telemetry);
        telemetry.update();
    }
}