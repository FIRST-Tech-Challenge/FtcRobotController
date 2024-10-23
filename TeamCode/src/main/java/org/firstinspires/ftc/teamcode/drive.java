package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.concurrent.TimeUnit;
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

    private DcMotorEx intakeInOut;
    private DcMotorEx clawUpDown;
    private double intakeMaxSpeed = 0.5;


    public void processIntakeInOut() {
        if (gamepad1.left_trigger == 0 && gamepad1.right_trigger == 0) {
            intakeInOut.setPower(0);
        }
        else if (gamepad1.left_trigger != 0 && gamepad1.right_trigger == 0){
            intakeInOut.setDirection(DcMotorSimple.Direction.REVERSE);
            if(gamepad1.left_trigger >= intakeMaxSpeed) {
                intakeInOut.setPower(intakeMaxSpeed);
            }
            else{
                intakeInOut.setPower(gamepad1.left_trigger);
            }

        }
        else if (gamepad1.right_trigger != 0 && gamepad1.left_trigger == 0) {
            intakeInOut.setDirection(DcMotorSimple.Direction.FORWARD);
            if (gamepad1.right_trigger >= intakeMaxSpeed) {
                intakeInOut.setPower(intakeMaxSpeed);
            } else {
                intakeInOut.setPower(gamepad1.right_trigger);
            }
        }
    }
    @Override
    public void init() {
        //Continue defining motors                            //rename in driver station to the below(expainson motor 0)
        intakeInOut = hardwareMap.get(DcMotorEx.class, "intakeinout");
        clawUpDown = hardwareMap.get(DcMotorEx.class, "clawupdown");
        // run once when init is pressed
        wifiSsid = WifiUtil.getConnectedSsid();

        drive.initHardware(hardwareMap, wifiSsid.equals(TESTBOT) ? MecanumEncoder.Bot.TestBot : MecanumEncoder.Bot.CompBot);
        intakeInOut.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeInOut.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
    }
}

