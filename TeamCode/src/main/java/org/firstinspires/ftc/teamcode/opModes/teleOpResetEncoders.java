package org.firstinspires.ftc.teamcode.opModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.nematocyst;
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrive;

@TeleOp
@Config

public class teleOpResetEncoders extends OpMode {
    public static double sP = 0.001;
    public static double sI = 0.000;
    public static double sD = 0.00;
    public double angP = 0.0045;
    public double angI = 0;
    public double angD = 0.00002;
    public double angCos = 0.325;
    public double angExt = 0.000075;
    nematocyst slide;
    org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrive SwerveDrive;
    Telemetry telemetry2;
    FtcDashboard dash;
    String[] encoderNames = {
            "fl_encoder",
            "fr_encoder",
            "bl_encoder",
            "br_encoder"
    };
    String[] driveNames = {
            "fl_drive",
            "fr_drive",
            "bl_drive",
            "br_drive"
    };
    String[] angleNames = {
            "fl_angle",
            "fr_angle",
            "bl_angle",
            "br_angle"
    };

    @Override
    public void init() {
        SwerveDrive = new SwerveDrive(
                18, 18, 12, 12,
                this, gamepad1, hardwareMap,
                encoderNames, driveNames, angleNames, 0,0,0);
        dash = FtcDashboard.getInstance();
        telemetry2 = dash.getTelemetry();
        slide = new nematocyst(this);
        slide.init("pivot", "slide", "wrist", "claw");
        slide.pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slide.slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }
    @Override
    public void init_loop () {
        SwerveDrive.init_loop();

    }
    @Override
    public void loop() {
        double[] xAndY = fieldCentricXandY(
                SwerveDrive.imu.getRobotYawPitchRollAngles().getYaw(), -gamepad1.left_stick_x, -gamepad1.left_stick_y);
        SwerveDrive.loop(xAndY[0], xAndY[1], (gamepad2.right_stick_x + gamepad1.right_stick_x)/2);
        // trying to separate field centricity into the opmode level
//        SwerveDrive.setPID(P, I, D);
        SwerveDrive.getTelemetry(telemetry2);
        telemetry.update();
        telemetry2.update();
        if (gamepad1.a) { SwerveDrive.resetIMU();}
        if (gamepad2.a || gamepad2.cross) {
            slide.goUp(0);
        } else if (gamepad2.x || gamepad2.square) {
            slide.goGround(13.0);
        } else if (gamepad2.b || gamepad2.circle) {
            slide.goSpecimen(28);
        } else if (gamepad2.y || gamepad2.triangle) {
            slide.goOut(36.0);
        } else if (gamepad2.right_trigger > 0.5) {
            slide.goSpecimenDown(2);
        }

        //TODO: Uncomment this last match
//        if (gamepad2.start) {
//            slide.hangSetup();
//        } else if (gamepad2.back) {
//            slide.hangReal();
//        }

        if (gamepad2.dpad_down) {
            slide.wristIn();
        } else if (gamepad2.dpad_up) {
            slide.wristOut();
        } else if (gamepad2.left_bumper) {
            slide.grab();
        } else if (gamepad2.right_bumper) {
            slide.release();
        } else if (gamepad2.dpad_left) {
            slide.wristDown();
        } else if (gamepad2.dpad_right) {
            slide.wristSpecimen();
        }
        slide.loop(sP, sI, sD);
        slide.updatePID(angP, angI, angD, angCos, angExt);
        slide.updateSlidePID(sP, sI, sD);
        slide.getTelemetry();
        slide.getTelemetry(telemetry2);
    }

    public double[] fieldCentricXandY(double theta, double x, double y) {
        double theta2 = Math.toRadians(theta);
        double fieldX = x * Math.cos(theta2) - y * Math.sin(theta2);
        double fieldY = x * Math.sin(theta2) + y * Math.cos(theta2);

        return new double[]{fieldX, fieldY};
    }
}
