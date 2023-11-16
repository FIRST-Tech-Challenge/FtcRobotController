package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.HydraPixelPalaceActions;

public class HydraPixelPalace {
    private final Servo mSrvPxlPos1;
    private final Servo mSrvPxlPos2;
    private final DistanceSensor mSenColPxlPos1_DistanceSensor;
    private final DistanceSensor mSenColPxlPos2_DistanceSensor;
    private final LED mLED1;
    private final LED mLED2;
    private final LED mLED3;
    private final LED mLED4;
    private final double mPxlSrvSpeedFrontToBack;
    private final double mPxlSrvSpeedBackToFront;
    private final double mPxlPos1DetDist;
    private final double mPxlPos2DetDist;

    public HydraPixelPalace(HardwareMap hardwareMap, String srvPos1, String srvPos2, String led1, String led2, String led3,
                            String led4, String snsPos1, String snsPos2, double srvSpeedFrontToBack,
                            double srvSpeedBackToFront, double pixelDistPos1, double pixelDistPos2) {
        mSrvPxlPos1 = hardwareMap.get(Servo.class, srvPos1);
        mSrvPxlPos2 = hardwareMap.get(Servo.class, srvPos2);
        ColorSensor mSenColPxlPos1 = hardwareMap.get(ColorSensor.class, snsPos1);
        mSenColPxlPos1_DistanceSensor = hardwareMap.get(DistanceSensor.class, snsPos1);
        ColorSensor mSenColPxlPos2 = hardwareMap.get(ColorSensor.class, snsPos2);
        mSenColPxlPos2_DistanceSensor = hardwareMap.get(DistanceSensor.class, snsPos2);
        mLED1 = hardwareMap.get(LED.class, led1);
        mLED2 = hardwareMap.get(LED.class, led2);
        mLED3 = hardwareMap.get(LED.class, led3);
        mLED4 = hardwareMap.get(LED.class, led4);
        mPxlSrvSpeedBackToFront = srvSpeedBackToFront;
        mPxlSrvSpeedFrontToBack = srvSpeedFrontToBack;
        mPxlPos1DetDist = pixelDistPos1;
        mPxlPos2DetDist = pixelDistPos2;

        // Set servo direction. Position 2 is reversed
        mSrvPxlPos1.setDirection(Servo.Direction.FORWARD);
        mSrvPxlPos2.setDirection(Servo.Direction.REVERSE);
        // Disable the LEDs since we only need distance measurements
        mSenColPxlPos1.enableLed(false);
        mSenColPxlPos2.enableLed(false);
        // Ensure the servos are stopped
        Stop();
    }

    public int Start(HydraPixelPalaceActions position1, HydraPixelPalaceActions position2, boolean score) {
        int pixelsDetected = PixelLocations();
        boolean pixelInPos1 = (pixelsDetected & 1) != 0;
        boolean pixelInPos2 = (pixelsDetected & 2) != 0;
        if (pixelInPos2) {
            if (position2 == HydraPixelPalaceActions.PixelPalaceFrontToBack && !score) {
                // Don't run off the end unless we are scoring on the backdrop
                position2 = HydraPixelPalaceActions.PixelPalaceStop;
            } else if (position2 == HydraPixelPalaceActions.PixelPalaceBackToFront &&
                    position1 != HydraPixelPalaceActions.PixelPalaceBackToFront) {
                // Don't push into position 1 if it's not moving in the same direction
                position2 = HydraPixelPalaceActions.PixelPalaceStop;
            }
        }
        if (pixelInPos1) {
            if (position1 == HydraPixelPalaceActions.PixelPalaceFrontToBack && pixelInPos2) {
                // Don't run a pixel into another pixel
                position1 = HydraPixelPalaceActions.PixelPalaceStop;
            }
        }
        mLED4.enable(pixelInPos2);
        mLED3.enable(!pixelInPos2);
        mLED2.enable(pixelInPos1);
        mLED1.enable(!pixelInPos1);
        SetPixelSrvDir(mSrvPxlPos1, position1);
        SetPixelSrvDir(mSrvPxlPos2, position2);
        return pixelsDetected;
    }

    private int PixelLocations() {
        boolean pixelInPos1 = DetectPixelPos1();
        boolean pixelInPos2 = DetectPixelPos2();
        int ret = 0;
        if (pixelInPos1) {
            ret |= 1;
        }
        if (pixelInPos2) {
            ret |= 2;
        }
        return ret;
    }

    public void Stop() {
        SetPixelSrvDir(mSrvPxlPos1, HydraPixelPalaceActions.PixelPalaceStop);
        SetPixelSrvDir(mSrvPxlPos2, HydraPixelPalaceActions.PixelPalaceStop);
    }

    /**
     * Describe this function...
     */
    private void SetPixelSrvDir(Servo srv, HydraPixelPalaceActions action) {
        // Keep Servo position in valid range
        double direction;
        switch (action) {
            default:
            case PixelPalaceStop:
                direction = 0.5;
                break;
            case PixelPalaceBackToFront:
                direction = mPxlSrvSpeedBackToFront;
                break;
            case PixelPalaceFrontToBack:
                direction = mPxlSrvSpeedFrontToBack;
                break;
        }
        direction = Math.min(Math.max(direction, 0), 1);
        srv.setPosition(direction);
        //telemetry.addData("PixelServo", direction);
    }

    /**
     * Describe this function...
     */
    private boolean DetectPixelPos1() {
        double distPixelPos1;
        boolean detPixelPos1;

        distPixelPos1 = mSenColPxlPos1_DistanceSensor.getDistance(DistanceUnit.CM);
        detPixelPos1 = distPixelPos1 < mPxlPos1DetDist;
        //telemetry.addData("PixelPos1Dist", distPixelPos1);
        //telemetry.addData("Pixel1Detect", detPixelPos1);
        return detPixelPos1;
    }

    /**
     * Describe this function...
     */
    private boolean DetectPixelPos2() {
        double distPixelPos2;
        boolean detPixelPos2;

        distPixelPos2 = mSenColPxlPos2_DistanceSensor.getDistance(DistanceUnit.CM);
        detPixelPos2 = distPixelPos2 < mPxlPos2DetDist;
        //telemetry.addData("PixelPos2Dist", distPixelPos2);
        //telemetry.addData("Pixel2Detect", detPixelPos2);
        return detPixelPos2;
    }
}
