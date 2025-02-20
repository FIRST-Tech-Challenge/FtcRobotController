package org.firstinspires.ftc.teamcode.TestingAndTuning;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name = "ColorTelemetry", group = "TESTING")
public class ColorTelemetry extends LinearOpMode {

    public static class RedSampleColors{
        public static final double RED_MAX = 0.017;
        public static final double GREEN_MAX = 0.009;
        public static final double BLUE_MAX = 0.0042;

        public static final double RED_MIN = 0.0018;
        public static final double GREEN_MIN = 0.0015;
        public static final double BLUE_MIN = 0.0009;
    }

    public static class YellowSampleColors{
        public static final double RED_MAX = 0.0206;
        public static final double GREEN_MAX = 0.0285;
        public static final double BLUE_MAX = 0.006;

        public static final double RED_MIN = 0.0027;
        public static final double GREEN_MIN = 0.0037;
        public static final double BLUE_MIN = 0.0013;
    }

    public static class BlueSampleColors{
        public static final double RED_MAX = 0.0028;
        public static final double GREEN_MAX = 0.0057;
        public static final double BLUE_MAX = 0.013;

        public static final double RED_MIN = 0.0008;
        public static final double GREEN_MIN = 0.0014;
        public static final double BLUE_MIN = 0.002;
    }

    public static class AmbientColors{
        public static final double RED_MAX = 0.0437;
        public static final double GREEN_MAX = 0.0735;
        public static final double BLUE_MAX = 0.033;

        public static final double RED_MIN = 0.0007;
        public static final double GREEN_MIN = 0.0015;
        public static final double BLUE_MIN = 0.0018;
    }


    public void runOpMode() {

        RevColorSensorV3 colorSens = hardwareMap.get(RevColorSensorV3.class, "intakeColor");
        NormalizedRGBA color = null;

        waitForStart();

        while(opModeIsActive()){
            color = colorSens.getNormalizedColors();

            telemetry.addData("Red: ", color.red);
            telemetry.addData("Green: ", color.green);
            telemetry.addData("Blue: ", color.blue);

            telemetry.addLine();telemetry.addLine();

            if(color.red > RedSampleColors.RED_MIN && color.red < RedSampleColors.RED_MAX && color.green < RedSampleColors.GREEN_MAX && color.blue < RedSampleColors.BLUE_MAX){
                telemetry.addLine("RED");
            }

            if(color.green > YellowSampleColors.GREEN_MIN && color.green < YellowSampleColors.GREEN_MAX && color.red < YellowSampleColors.RED_MAX && color.blue < YellowSampleColors.BLUE_MAX){
                telemetry.addLine("YELLOW");
            }
            if(color.blue > BlueSampleColors.BLUE_MIN && color.blue < BlueSampleColors.BLUE_MAX && color.red < BlueSampleColors.RED_MAX && color.green < BlueSampleColors.GREEN_MAX){
                telemetry.addLine("BLUE");
            }

            telemetry.addLine();

            telemetry.update();
        }
    }
}
