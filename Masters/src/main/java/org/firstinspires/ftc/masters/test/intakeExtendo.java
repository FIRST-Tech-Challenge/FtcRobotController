package org.firstinspires.ftc.masters.test;

import static org.firstinspires.ftc.masters.components.Intake.INTAKE_POWER;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.masters.components.DriveTrain;
import org.firstinspires.ftc.masters.components.ITDCons;
import org.firstinspires.ftc.masters.components.Init;
import org.firstinspires.ftc.masters.components.Intake;

@Config // Enables FTC Dashboard
@TeleOp(name = "Intake extend")
public class intakeExtendo extends LinearOpMode {

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        Init init = new Init(hardwareMap);
        Intake intake = new Intake(init, telemetry);

        telemetry.update();

        intake.intakeToNeutral();

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.a) {
                intake.extendSlideMax();
            }
            if (gamepad1.b) {
                intake.extendSlideHalf();
            }
            if (gamepad1.x) {
                intake.retractSlide();
            }

            intake.update();

        }

    }
}

