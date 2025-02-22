package org.firstinspires.ftc.teamcode.TestingAndTuning;

import android.graphics.Color;
import android.graphics.ColorSpace;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.Arrays;

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

    public static final double RED_SAMPLE_HUE = 25.0;
    public static final double YELLOW_SAMPLE_HUE = 80.0;
    public static final double BLUE_SAMPLE_HUE = 210.0;
    public static final double AMBIENT_HUE = 190;

    ColorSensor colorSens;

    float hsv[] = {0,0,0};

    ElapsedTime colorTimer;


    public void runOpMode() {

        colorTimer = new ElapsedTime();

        colorSens = hardwareMap.get(ColorSensor.class, "intakeColor");


        waitForStart();


        while(opModeIsActive()){
            if(colorTimer.milliseconds() > 100) telemetry.addLine(getSampleColor(3));
            telemetry.update();

//            color = new Color(colorSens.red(), colorSens.green(), colorSens.blue());

//            telemetry.addData("Hue: ", hsv[0]);
//            telemetry.addData("Saturation: ", hsv[1]);
//            telemetry.addData("Value: ", hsv[2]);
//
//            telemetry.addLine();telemetry.addLine();
//
//            telemetry.addData("Red: ", colorSens.red());
//            telemetry.addData("Green: ", colorSens.green());
//            telemetry.addData("Blue: ", colorSens.blue());

//            telemetry.addLine();telemetry.addLine();

//            if(color.red > RedSampleColors.RED_MIN && color.red < RedSampleColors.RED_MAX && color.green < RedSampleColors.GREEN_MAX && color.blue < RedSampleColors.BLUE_MAX){
//                telemetry.addLine("RED");
//            }
//
//            if(color.green > YellowSampleColors.GREEN_MIN && color.green < YellowSampleColors.GREEN_MAX && color.red < YellowSampleColors.RED_MAX && color.blue < YellowSampleColors.BLUE_MAX){
//                telemetry.addLine("YELLOW");
//            }
//            if(color.blue > BlueSampleColors.BLUE_MIN && color.blue < BlueSampleColors.BLUE_MAX && color.red < BlueSampleColors.RED_MAX && color.green < BlueSampleColors.GREEN_MAX){
//                telemetry.addLine("BLUE");
//            }

//            telemetry.addLine();

//            telemetry.update();
        }
    }
    private String getClosestHue(double measuredHue){
        final double[] COLOR_ERRORS = {Math.abs(measuredHue-RED_SAMPLE_HUE), Math.abs(measuredHue-YELLOW_SAMPLE_HUE), Math.abs(measuredHue-BLUE_SAMPLE_HUE), Math.abs(measuredHue-AMBIENT_HUE)};
//      {redError, yellowError, blueError, ambientError}

        double min = Double.MAX_VALUE;
        int minIndex = Integer.MAX_VALUE;
        for(int i = 0; i < COLOR_ERRORS.length; i++){
            if(COLOR_ERRORS[i] < min){
                min = COLOR_ERRORS[i];
                minIndex = i;
            }
        }
        switch (minIndex){
            case(0):
                return "RED";
            case(1):
                return "YELLOW";
            case(2):
                return "BLUE";
            default:
                return "NONE";
        }

    }

    public String getSampleColor(int numReps){
        double[] hues = new double[numReps];
        int i = 0;
        hues = getColorHueRecursive(hues, i, numReps);
        Arrays.sort(hues);
        double median = hues[numReps/2];
        return getClosestHue(median);
    }

    public double[] getColorHueRecursive(double[] huesCopy, int index, int numReps){
        if(index >= numReps-1){
            telemetry.addData("Sample " + (index+1) + ": ", huesCopy[index]);
            return huesCopy;
        } else{
            if(index < numReps && colorTimer.milliseconds() > 10) {
                colorTimer.reset();
                index++;
                Color.RGBToHSV(colorSens.red(), colorSens.green(), colorSens.blue(), hsv);
                huesCopy[index] = hsv[0];
                telemetry.addData("Sample " + index + ": ", huesCopy[index]);
            }
            return getColorHueRecursive(huesCopy, index, numReps);
        }
    }

}
