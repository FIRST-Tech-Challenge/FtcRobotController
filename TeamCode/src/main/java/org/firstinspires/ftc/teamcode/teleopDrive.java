package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.internal.network.WifiUtil;
import org.firstinspires.ftc.teamcode.hardware.MecanumEncoder;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Slide;

@TeleOp
public class teleopDrive extends OpMode {

    private String TESTBOT = "24342-RC";
    private Telemetry.Item telPathDebug = null;
    private MecanumEncoder drive = new MecanumEncoder(this);
    private Slide intakeSlide = new Slide("intakeslide", "", Slide.ExtendMotorDirection.Forward, 1300, 1.0, 114.28);
    private Slide clawSlide = new Slide("clawslide", "", Slide.ExtendMotorDirection.Reverse, 4500, 1.0,114.28);
    private String wifiSsid = "";

    public void processClawArmUpDown() {
        if (gamepad2.left_trigger == 0 && gamepad2.right_trigger == 0) {
            clawSlide.Stop();
        }
        else if (gamepad2.left_trigger != 0 && gamepad2.right_trigger == 0){
            clawSlide.Retract(gamepad2.left_trigger);
        }
        else if (gamepad2.right_trigger != 0 && gamepad2.left_trigger == 0) {
            clawSlide.Extend(gamepad2.right_trigger);
        }
    }
    public void processIntakeInOut() {
        if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0) {
            intakeSlide.Stop();
        }
        else if (gamepad1.left_trigger != 0 && gamepad1.right_trigger == 0){
            intakeSlide.Retract(gamepad1.left_trigger);
        }
        else if (gamepad1.right_trigger != 0 && gamepad1.left_trigger == 0) {
            intakeSlide.Extend(gamepad1.right_trigger);
        }
    }
    @Override
    public void init() {
        //Continue defining motors
        intakeSlide.Init(hardwareMap);
        clawSlide.Init(hardwareMap);
        // run once when init is pressed
        wifiSsid = WifiUtil.getConnectedSsid();

        drive.initHardware(hardwareMap, wifiSsid.equals(TESTBOT) ? MecanumEncoder.Bot.TestBot : MecanumEncoder.Bot.CompBot);
        telemetry.clearAll();
        telemetry.setAutoClear(false);
        telPathDebug = telemetry.addData("PathDebug:", "");




    }

    @Override
    public void init_loop() {
        // add stuff here for the init loop
        telPathDebug.setValue(wifiSsid);
        telemetry.update();
    }

    @Override
    public void loop() {
        // runs while in play
        drive.driverInput(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x, 1.0, MecanumEncoder.DriveMode.FieldCentric);
        processIntakeInOut();
        processClawArmUpDown();
    }
}

