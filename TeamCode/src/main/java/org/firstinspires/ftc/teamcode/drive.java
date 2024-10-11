package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.internal.network.WifiUtil;
import org.firstinspires.ftc.teamcode.hardware.MecanumEncoder;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.system.SystemProperties;

@TeleOp
public class drive extends OpMode {

    private String TESTBOT = "24342-RC";
    private Telemetry.Item telPathDebug = null;
    private MecanumEncoder drive = new MecanumEncoder(this);
    private String wifiSsid = "";

    @Override
    public void init() {
        // run once when init is pressed
        wifiSsid = WifiUtil.getConnectedSsid();

        drive.initHardware(hardwareMap, wifiSsid.equals(TESTBOT) ? MecanumEncoder.Bot.TestBot : MecanumEncoder.Bot.CompBot);
        drive.resetYaw();

        telemetry.clearAll();
        telemetry.setAutoClear(false);
        telPathDebug = telemetry.addData("PathDebug:", "");
    }

    @Override
    public void init_loop() {
        // add stuff here for the init loop
    }

    @Override
    public void loop() {
        // runs while in play
        drive.driverInput(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 1.0, MecanumEncoder.DriveMode.FieldCentric);

        telPathDebug.setValue(wifiSsid);
    }
}

