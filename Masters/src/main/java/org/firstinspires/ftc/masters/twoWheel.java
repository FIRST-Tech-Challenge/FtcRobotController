package org.firstinspires.ftc.masters;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Config // Enables FTC Dashboard
@TeleOp(name = "Two Wheel")
public class twoWheel extends LinearOpMode {


    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public static double Blank = 0;

    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        DcMotor leftmoter = hardwareMap.dcMotor.get("leftmoter");
        DcMotor rightmoter = hardwareMap.dcMotor.get("rightmoter");

        waitForStart();

        while (opModeIsActive()) {

            double yl = gamepad1.left_stick_y * 0.5;
            double yr = gamepad1.right_stick_y * 0.5;

            telemetry.addData("yl", yl);
            telemetry.addData("yr", yr);
            telemetry.update();

            leftmoter.setPower(-yl);
            rightmoter.setPower(yr);

        }
    }
}

