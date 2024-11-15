package org.firstinspires.ftc.teamcode.tests;

import androidx.annotation.RequiresPermission;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;

@TeleOp
public class voltageToAngleTest extends OpMode {
    AnalogInput flEnc, frEnc, blEnc, brEnc;
    CRServo flAngle, frAngle, blAngle, brAngle;
    FtcDashboard dashboard;
    Telemetry telemetry2;
    @Override
    public void init() {
        flEnc = hardwareMap.get(AnalogInput.class, "fl_encoder");
        frEnc = hardwareMap.get(AnalogInput.class, "fr_encoder");
        blEnc = hardwareMap.get(AnalogInput.class, "bl_encoder");
        brEnc = hardwareMap.get(AnalogInput.class, "br_encoder");

//
//        flAngle = hardwareMap.get(CRServo.class, "fl_angle");
//        frAngle = hardwareMap.get(CRServo.class, "fr_angle");
//        blAngle = hardwareMap.get(CRServo.class, "bl_angle");
//        brAngle = hardwareMap.get(CRServo.class, "br_angle");
        ReadWriteFile.writeFile(new File("~/FIRST/wheelAngles.txt"), "");

        dashboard = FtcDashboard.getInstance();
        telemetry2 = dashboard.getTelemetry();
    }

    @Override
    public void loop() {

        telemetry.addData("flEnc", flEnc.getVoltage());
        telemetry2.addData("flEnc", flEnc.getVoltage());
        telemetry.addData("frEnc", frEnc.getVoltage());
        telemetry2.addData("frEnc", frEnc.getVoltage());
        telemetry.addData("blEnc", blEnc.getVoltage());
        telemetry2.addData("blEnc", blEnc.getVoltage());
        telemetry.addData("brEnc", brEnc.getVoltage());
        telemetry2.addData("brEnc", brEnc.getVoltage());

        telemetry.update();
        telemetry2.update();
    }
}
