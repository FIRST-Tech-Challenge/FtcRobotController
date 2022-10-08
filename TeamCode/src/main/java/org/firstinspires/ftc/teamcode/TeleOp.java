package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.util.IMU_EnhancedLocalizer;
import org.firstinspires.ftc.teamcode.util.Localizer;
import org.firstinspires.ftc.teamcode.util.MecanumDriveBase;


@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name="TeleOpStub", group="Basic drive")
public class TeleOp extends OpMode

{
    private IMU_EnhancedLocalizer IMUlocalizer = null;
    private MecanumDriveBase mecanumDriveBase = null;
    private PacManTurnToPos pacMan;

    public void init() {
        telemetry.addData("Status", "Initialized");
        IMUlocalizer = new IMU_EnhancedLocalizer(hardwareMap);
        mecanumDriveBase = new MecanumDriveBase(hardwareMap);
//        pacMan = new PacManTurnToPos(IMUlocalizer, mecanumDriveBase);
//        IMUlocalizer.gyroCalibrate();
        double max;
    }
    @Override
    public void loop() {
        IMUlocalizer.displayTelemetry(telemetry);
        IMUlocalizer.handleTracking();
        mecanumDriveBase.gamepadController(gamepad1);
        mecanumDriveBase.driveBaseTelemetry(telemetry);
//        pacMan.handlePacMan(gamepad1, telemetry);
        telemetry.update();

    }
}
