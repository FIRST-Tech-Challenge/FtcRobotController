package org.firstinspires.ftc.teamcode.samples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
 * Demonstration of the dashboard's versatile field overlay display capabilities.
 */
@Config
@Autonomous
public class FieldVersatilityOpMode extends LinearOpMode {
    public static double AMPLITUDE = 1;
    public static double PHASE = 90;
    public static double FREQUENCY = 0.25;
    public static double ORIGIN_OFFSET_X = 0;
    public static double ORIGIN_OFFSET_Y = 12 * 6;
    public static double ORIGIN_ZEROHEADING = Math.PI/2;
    public static boolean RED_ALLIANCE = true;
    public static double ORBITAL_FREQUENCY = 0.05;
    public static double SPIN_FREQUENCY = 0.25;
    public static double ORBITAL_RADIUS = 50;
    public static double SIDE_LENGTH = 10;
    public static String ALTIMGSRC = "https://upload.wikimedia.org/wikipedia/commons/4/45/Football_field.svg";
    public static double ALTIMGX = 0; //try 24
    public static double ALTIMGY = 0; //try 24
    public static double ALTIMGW = 144; //try 48
    public static double ALTIMGH = 144; //try 48
    public static boolean ALTIMGOPAQUE = true;
    public static double SCALEX = 1.0;
    public static double SCALEY = 1.0;
    public static double GRIDHORIZONTAL = 7; //includes field edges
    public static double GRIDVERTICAL = 7;

    private static void rotatePoints(double[] xPoints, double[] yPoints, double angle) {
        for (int i = 0; i < xPoints.length; i++) {
            double x = xPoints[i];
            double y = yPoints[i];
            xPoints[i] = x * Math.cos(angle) - y * Math.sin(angle);
            yPoints[i] = x * Math.sin(angle) + y * Math.cos(angle);
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        FtcDashboard dashboard = FtcDashboard.getInstance();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double time = getRuntime();

            double bx = ORBITAL_RADIUS * Math.cos(2 * Math.PI * ORBITAL_FREQUENCY * time);
            double by = ORBITAL_RADIUS * Math.sin(2 * Math.PI * ORBITAL_FREQUENCY * time);
            double l = SIDE_LENGTH / 2;
            //drawing an orbiting triangle pointing up the X axis to indicate field theta = 0
            double[] bxPoints = { 0, SIDE_LENGTH*2, 0 };
            double[] byPoints = { l, 0, -l };
            //rotatePoints(bxPoints, byPoints, 2 * Math.PI * SPIN_FREQUENCY * time);
            for (int i = 0; i < 3; i++) {
                bxPoints[i] += bx;
                byPoints[i] += by;
            }
            telemetry.addData("x", AMPLITUDE * Math.sin(
                    2 * Math.PI * FREQUENCY * (System.currentTimeMillis() / 1000d) + Math.toRadians(PHASE)
            ));
            telemetry.update();

            //draw the field overlay
            TelemetryPacket packet = new TelemetryPacket();
            packet.fieldOverlay()
                    //optionally add an alternate field image on top of the default
                    .setAltImage(ALTIMGSRC, ALTIMGX, ALTIMGY,ALTIMGW, ALTIMGH, ALTIMGOPAQUE)
                    //.setAltImage("", 0, 0,144, 144, false) //empty src will clear the alt field image

                    //optionally override default gridlines, minimum of 2 to render field edges, anything less suppresses gridlines in that direction, default is 7
                    .setGrid(GRIDHORIZONTAL, GRIDVERTICAL)

                    //historical default origin for dashboard is in the center of the field with X axis pointing up
                    //for powerplay season iron reign decided to set the origin to the alliance substation
                    //to take advantage of the inherent symmetries of the challenge:
                    .setRotation(RED_ALLIANCE ? 0: Math.PI)
                    .setTranslation(ORIGIN_OFFSET_X, ORIGIN_OFFSET_Y * (RED_ALLIANCE ? -1: 1))
                    //blue alliance would be
                    //.setRotation(Math.PI)
                    //.setOrigin(0, 12*6)

                    //.setRotation(-Math.PI/4) //uncomment to see a rotation of 45 degrees, there have been challenges with a diagonal field symmetry

                    .setScale(SCALEX, SCALEY) //be sure the vales evaluate to a doubles and not ints
                    //.setScale(144.0/105,144.0/105) //example of FIFA soccer field in meters

                    .setStrokeWidth(1)
                    //draw the axes of the new origin
                    .setStroke("red")
                    .strokeLine(0,0,24,0) //x axis
                    .setFill("red")
                    .fillText("X axis", 0, 0,"8px Arial", 0)
                    .setStroke("green")
                    .strokeLine(0,0,0,24) //y axis
                    .setFill("green")
                    .strokeText("Y axis", (RED_ALLIANCE? -24: 0), 0,"8px serif", Math.PI/2 * (RED_ALLIANCE? -1: 1))
                    .setStroke("goldenrod")
                    .strokeCircle(0, 0, ORBITAL_RADIUS)
                    .setFill("black")
                    .fillPolygon(bxPoints, byPoints);
            dashboard.sendTelemetryPacket(packet);

            sleep(20);
        }
    }
}
