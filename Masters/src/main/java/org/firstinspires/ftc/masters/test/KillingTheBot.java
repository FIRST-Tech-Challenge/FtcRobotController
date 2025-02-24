package org.firstinspires.ftc.masters.test;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.masters.components.DriveTrain;
import org.firstinspires.ftc.masters.components.Init;
import org.firstinspires.ftc.masters.components.Intake;
import org.firstinspires.ftc.masters.components.Outtake;

@Config // Enables FTC Dashboard
@TeleOp(name = "Bot Killer 3000")
public class KillingTheBot extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static int offset = 0;
    public static boolean isReseting = false;

    Servo led;
    DcMotor outtakeSlideLeft;
    DcMotor outtakeSlideRight;
    VoltageSensor voltageSensor;

    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Init init = new Init(hardwareMap);
        Outtake outtake = new Outtake(init, telemetry);

        led = init.getLed();
        outtakeSlideLeft = init.getOuttakeSlideLeft();
        outtakeSlideRight = init.getOuttakeSlideRight();
        voltageSensor = init.getVoltageSensor();

        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a){
                isReseting = true;
                outtakeSlideLeft.setPower(-1);
                outtakeSlideRight.setPower(-1);
            }

            if (gamepad1.b){
                outtake.setTarget(7000 - offset);
            }

            if (voltageSensor.getVoltage() < 10 && isReseting){
                outtakeSlideLeft.setPower(0);
                outtakeSlideRight.setPower(0);
                led.setPosition(1);
                offset = outtakeSlideRight.getCurrentPosition();
                isReseting = false;
            }

            if (!isReseting) {
                outtake.update();
            }

            telemetry.addData("Voltage", voltageSensor.getVoltage());
            telemetry.update();

        }
    }
}

