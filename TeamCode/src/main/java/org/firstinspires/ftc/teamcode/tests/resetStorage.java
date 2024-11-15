package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.apache.commons.lang3.ArrayUtils;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.swerve.voltageToAngleConstants;

import java.io.File;
import java.util.Arrays;

@TeleOp
public class resetStorage extends OpMode {
    voltageToAngleConstants angles;
    FtcDashboard dashboard;
    Telemetry telemetry2;
    File filePath;
    String[] encoders = {
            "fl_encoder",
            "fr_encoder",
            "bl_encoder",
            "br_encoder"
    };
    String[] writingFinal;
    String[] smallAngles;
    double[] offsetsNegative;
    String[] offsetsString;
    String[] zeroes = {
            "360.0",
            "360.0",
            "360.0",
            "360.0",
            "0",
            "0",
            "0",
            "0"
    };
    @Override
    public void init() {
//        try {
        angles = new voltageToAngleConstants(this, hardwareMap, encoders);
//        } catch (IOException e) {
//            throw new RuntimeException(e);
//        }
        offsetsNegative = new double[4];
        offsetsString = new String[4];
        dashboard = FtcDashboard.getInstance();
        telemetry2 = dashboard.getTelemetry();
        filePath = angles.dataLog;
        smallAngles = new String[4];
        angles.init_loop();
    }
    public void init_loop() {
        angles.loop();
        for (int i = 0; i < angles.sm.length; i++) {
            smallAngles[i] = Double.toString(angles.sm[i]);
            offsetsNegative[i] = -1 * (angles.sm[i]);
            offsetsString[i] = Double.toString(offsetsNegative[i]);
        }
        writingFinal = ArrayUtils.addAll(zeroes, smallAngles);
        writingFinal = ArrayUtils.addAll(writingFinal, offsetsString); // small angles again is the offsets that get added
    }
    @Override
    public void loop() {
        ReadWriteFile.writeFile(filePath, Arrays.toString(writingFinal));
    }
}
