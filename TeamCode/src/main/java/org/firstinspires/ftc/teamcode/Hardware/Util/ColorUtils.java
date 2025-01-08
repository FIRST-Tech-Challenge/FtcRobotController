package org.firstinspires.ftc.teamcode.Hardware.Util;

import org.firstinspires.ftc.teamcode.SystemsFSMs.Mechaisms.SampleDetector;

public class ColorUtils {

    public static double[] RGBAtoHSV(double r, double g, double b, double a) {
        //Inputs are r,g,b [0,255] & a [0,1]
        //Returns array H [0,255], S [0,100], V [0,100], in array

        // RGBA to RGB
        double[] backgroundColor = {255, 255, 255, 1};
        double aMix = a + (backgroundColor[3] * (1 - a));

        double rMix = ((r * a) + (backgroundColor[0] * backgroundColor[3] * (1 - a))) / aMix;
        double gMix = ((g * a) + (backgroundColor[1] * backgroundColor[3] * (1 - a))) / aMix;
        double bMix = ((b * a) + (backgroundColor[2] * backgroundColor[3] * (1 - a))) / aMix;

        //RGB to HSV
        double rangedR = rMix / 255;
        double rangedG = gMix / 255;
        double rangedB = bMix / 255;

        double max = Math.max(rangedR, Math.max(rangedG, rangedB));
        double min = Math.min(rangedR, Math.max(rangedG, rangedB));
        double delta = max - min;

        double h = 0;
        double s = 0;
        double v = 0;

        // Calculating Hue, in [0,255]
        if (max == rangedR) {
            h = 60 * ((rangedG - rangedB) / delta % 6);
        } else if (max == rangedG) {
            h = 60 * ((rangedB - rangedR) / delta + 2);
        } else if (max == rangedB) {
            h = 60 * ((rangedR - rangedG) / delta + 4);
        }

        //Angle Wraparound
        if (Math.signum(h) == -1) {
            h = 360 + h;
        }

        // Convert from [0,360] to [0,255]
        //h = (255 * h) / 360;

        // Calculating Saturation, in [0,100]
        if (max != 0) {
            s = 100 * (delta / max);
        }

        // Calculating Value, in [0,100]
        v = 100 * max;

        return new double[] {h, s, v};

    }

    public static double HSVDistance(double[] color1, double[] color2) {

        double h1 = color1[0];
        double h2 = color2[0];
        double s1 = color1[1];
        double s2 = color2[1];
        double v1 = color1[2];
        double v2 = color2[2];

        double hDist = (Math.min(Math.abs(h2 - h1), 360 - (Math.abs(h2 - h1))) / 360) * 2 * Math.PI * 100;
        double sDist = Math.abs(s2-s1);
        double vDist = Math.abs(v2-v1);

        double distance = Math.sqrt(Math.pow(hDist, 2) + Math.pow(sDist, 2) + Math.pow(vDist, 2));

        return distance;
    }

    public static double Clamp(double value, double min, double max){
        return Math.min(Math.max(value, min), max);
    }

    public static double Normalize(double value, double normalMax, double valueMax) {
        return (value * normalMax) / valueMax;
    }

    public static SampleDetector.SampleColor ClassifyColor(double[] color) {
        // Takes an HSV Color [0,255], [0,100], [0,100]

        double[] yellowHSV = {64, 95, 95};
        double[] blueHSV = {224, 96, 96};
        double[] redHSV = {17, 91, 92};

        double yellowDist = HSVDistance(color, yellowHSV);
        double blueDist = HSVDistance(color, blueHSV);
        double redDist = HSVDistance(color, redHSV);

        double minDist = Math.min(yellowDist, Math.min(blueDist, redDist));

        if ( minDist == yellowDist ) {
            return SampleDetector.SampleColor.yellow;
        } else if (minDist == blueDist) {
            return SampleDetector.SampleColor.blue;
        } else if (minDist == redDist) {
            return SampleDetector.SampleColor.red;
        } else {
            return SampleDetector.SampleColor.unknown;
        }
    }



}
