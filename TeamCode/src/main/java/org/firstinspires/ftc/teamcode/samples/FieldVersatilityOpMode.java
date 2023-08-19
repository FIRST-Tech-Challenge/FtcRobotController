package org.firstinspires.ftc.teamcode.samples;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * Demonstration of the dashboard's versatile field overlay display capabilities.
 */
@Config
@TeleOp
public class FieldVersatilityOpMode extends LinearOpMode {
    public static double AMPLITUDE = 1;
    public static double PHASE = 90;
    public static double FREQUENCY = 0.25;
    public static double ORIGIN_OFFSET_X = 0;
    public static double ORIGIN_OFFSET_Y = 12 * 6;
    public static double ORIGIN_ZEROHEADING = Math.PI / 2;
    public static boolean RED_ALLIANCE = true;
    public static double ORBITAL_FREQUENCY = 0.05;
    public static double SPIN_FREQUENCY = 0.25;
    public static double ORBITAL_RADIUS = 50;
    public static double SIDE_LENGTH = 10;
    public static String ALTIMGSRC = "https://upload.wikimedia.org/wikipedia/commons/4/45/Football_field.svg";
    //public static String ALTIMGSRC = "dist/assets/play_arrow.95e2d7e4.svg";
    public static double ALTIMGX = 0; //try 24
    public static double ALTIMGY = 0; //try 24
    public static double ALTIMGW = 144; //try 48
    public static double ALTIMGH = 144; //try 48
    public static double SCALEX = 1.0;
    public static double SCALEY = 1.0;

    public static double LOGOX = 24, LOGOY = 24, LOGOW = 48, LOGOH = 48, LOGO_PIVOTX = 24, LOGO_PIVOTY = 24, LOGO_THETA_DEGREES = 15;
    public static boolean LOGO_USE_PAGE_FRAME = false;
    public static int GRID_LINESX = 13; //includes field edges
    public static int GRID_LINESY = 13;
    public static double GRIDX = -24;
    public static double GRIDY = 24;
    public static double GRIDH = 48;
    public static double GRIDW = 48;
    public static double GRID_THETA_DEGREES = 45;
    public static double GRID_PIVOTX = 24;
    public static double GRID_PIVOTY = 24;
    public static boolean GRID_USE_PAGE_FRAME = false;
    public static boolean DRAW_DEFAULT_FIELD = false;
    public static String TXTTEXT = "baseline";
    public static String TXTFONT = "8px arial";
    public static double TXTX = 48;
    public static double TXTY = 48;
    public static double TXT_THETA_DEGREES = 90;
    public static boolean TXT_USE_PAGE_FRAME = false;


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
        telemetry = dashboard.getTelemetry();

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            double time = getRuntime();

            double bx = ORBITAL_RADIUS * Math.cos(2 * Math.PI * ORBITAL_FREQUENCY * time);
            double by = ORBITAL_RADIUS * Math.sin(2 * Math.PI * ORBITAL_FREQUENCY * time);
            double l = SIDE_LENGTH / 2;

            double angleAnim;
            long millis = System.currentTimeMillis();
            double seconds = millis / 1000.0 * SPIN_FREQUENCY;
            double fraction = seconds - (int) seconds;
            angleAnim = 2 * Math.PI * fraction;

            //drawing an orbiting triangle pointing mostly up the X axis to indicate field theta = 15 degrees counter clockwise
            double[] bxPoints = {0, SIDE_LENGTH * 2, 0};
            double[] byPoints = {l, 0, -l};
            //rotatePoints(bxPoints, byPoints, 2 * Math.PI * SPIN_FREQUENCY * time);
            rotatePoints(bxPoints, byPoints, Math.toRadians(15));
            for (int i = 0; i < 3; i++) {
                bxPoints[i] += bx;
                byPoints[i] += by;
            }
            telemetry.addData("x", AMPLITUDE * Math.sin(
                    2 * Math.PI * FREQUENCY * (System.currentTimeMillis() / 1000d) + Math.toRadians(PHASE)
            ));
            telemetry.addData("theta", angleAnim);
            telemetry.update();

            //draw the field overlay
            TelemetryPacket packet = new TelemetryPacket(DRAW_DEFAULT_FIELD);

            packet.fieldOverlay()
                    //explicitly draw another field image
                    //images, text and grids by default are drawn using the page transform where 0,0 is top left and 144,144 is bottom right
                    //but extra parameters are available to allow rotation and drawing in the current transform
                    //all other drawing primitives are rendered in the current (or default) transform, as built using setTranslation, setRotation and setScale
                    .setAlpha(.25)
                    .drawImage(ALTIMGSRC, ALTIMGX, ALTIMGY, ALTIMGW, ALTIMGH)

                    //optionally add custom gridlines, minimum of 2 to render field edges, anything less suppresses gridlines in that direction, default is 7
                    .setAlpha(1.0)
                    //this is how to draw the default grid if you disable the default field
                    //this will be drawn in the pageFrame orientation
                    .drawGrid(0, 0, 144, 144, 7, 7)
                    //disabling the pageFrame will draw a grid in the current transform
                    .drawGrid(GRIDX, GRIDY, GRIDW, GRIDH, GRID_LINESX, GRID_LINESY, Math.toRadians(GRID_THETA_DEGREES), GRID_PIVOTX, GRID_PIVOTY, GRID_USE_PAGE_FRAME)
                    //.drawGrid(GRIDX, GRIDY, GRIDW, GRIDH, GRID_LINESX, GRID_LINESY, angleAnim, GRID_PIVOTX, GRID_PIVOTY, GRID_USE_PAGE_FRAME)

                    //you can draw multiple images and can rotate them around a specified anchor/pivot point and draw them in the current transform instead of the page frame
                    .drawImage("/dash/ftc.jpg", LOGOX, LOGOY, LOGOW, LOGOH, Math.toRadians(LOGO_THETA_DEGREES), LOGO_PIVOTX, LOGO_PIVOTY, LOGO_USE_PAGE_FRAME)

                    //demonstrate an alternate transform to move the origin and orientation
                    //default origin for dashboard is in the center of the field with X axis pointing up
                    //for powerplay season iron reign decided to set the origin to the alliance substation
                    //to take advantage of the inherent symmetries of the challenge:
                    .setRotation(RED_ALLIANCE ? 0 : Math.PI)
                    .setTranslation(ORIGIN_OFFSET_X, ORIGIN_OFFSET_Y * (RED_ALLIANCE ? -1 : 1))

                    //.setRotation(-Math.PI/4) //uncomment to see a rotation of 45 degrees, there have been FTC games with a diagonal field symmetry

                    .setScale(SCALEX, SCALEY) //be sure the vales evaluate to a doubles and not ints
                    //.setScale(144.0/105,144.0/105) //example of FIFA soccer field in meters

                    //draw the axes of the new origin
                    //the text labels will be drawn in the current origin transform, not in the page frame
                    .setStrokeWidth(1)
                    .setStroke("green")
                    .strokeLine(0, 0, 0, 24) //y axis
                    .setFill("green")
                    .strokeText("Y axis", 0, (RED_ALLIANCE ? 24 : 0), "8px serif", -Math.PI / 2 * (RED_ALLIANCE ? -1 : 1), false)
                    .setStroke("red")
                    .strokeLine(0, 0, 24, 0) //x axis
                    .setFill("red")
                    .fillText("X axis", 0, 0, "8px Arial", 0, false)

                    .setStroke("goldenrod")
                    .strokeCircle(0, 0, ORBITAL_RADIUS)
                    .setFill("black")
                    .fillPolygon(bxPoints, byPoints)
                    .setFill("blue")
                    //label the arrow as pointing 15 degree counter clockwise
                    .fillText("15 deg CC", bx - 10, by, "8px Arial", Math.toRadians(90 - 15), false)
                    .setAlpha(.25)
                    //you can draw multiple images and can rotate them around a specified pivot point, and draw them in the current transform instead of the page frame
                    .drawImage("/dash/powerplay.png", 24, 24, 48, 48, angleAnim, 24, 24, false)
                    .setAlpha(1.0)
                    .fillText(TXTTEXT, TXTX, TXTY, TXTFONT, Math.toRadians(TXT_THETA_DEGREES), TXT_USE_PAGE_FRAME);

            dashboard.sendTelemetryPacket(packet);

            sleep(20);
        }
    }
}
