package org.firstinspires.ftc.teamcode.samples;

import android.graphics.fonts.Font;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Autonomous
@Config
public class FlysetDemoOpMode extends LinearOpMode {
    public static String imageSource = "data:image/jpeg;base64,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";
    public static boolean drawField = true;
    public static double rectangleX = 0, rectangleY = 0, rectangleWidth = 6, rectangleHeight = 12;
    public static double originX, originY;
    public static double scaleX = 1, scaleY = 1;
    public static int gridLinesX = 2,gridLinesY = 2;
    public static double rotationDegrees;
    public static int originCircleRadius;
    public static String rectangleColor = "black";
    public static String originCircleColor = "red";

    public static String text = "";
    public static int imageX, imageY = 0;
    public static String textColor = "black";
    public static boolean imageEnabled = false;
    public static int imageHeight = 48, imageWidth = 72;
    public static String textFont = "12px arial";
    public static int textX, textY = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        waitForStart();
        int imageheight = imageHeight;

        while(opModeIsActive()) {
            if(!imageEnabled) {
                imageheight = 0;
            }
            else {
                imageheight = imageHeight;
            }
            TelemetryPacket packet = new TelemetryPacket(drawField);
            packet.fieldOverlay()
                .setScale(scaleX, scaleY)
                .drawGrid(0, 0, 144, 144, gridLinesX, gridLinesY)
                .setRotation(Math.toRadians(rotationDegrees))
                .setTranslation(originX, originY)
                .setFill(textColor)
                .drawImage(imageSource, imageX, imageY, imageheight, imageWidth)
                .fillText(text, textX, textY, textFont, 0)
                .setStroke(originCircleColor)
                .strokeCircle(0, 0, originCircleRadius)
                    .setStroke(rectangleColor)
                    .strokeRect(rectangleX, rectangleY, rectangleWidth, rectangleHeight)
                ;

            dashboard.sendTelemetryPacket(packet);

        }
    }
}
