package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.types.HydraPixelPalaceActions;

public class HydraPixelPalace {
    protected final Servo mSrvPxlPos1;
    protected final Servo mSrvPxlPos2;
    protected final DistanceSensor mSenColPxlPos1_DistanceSensor;
    protected final DistanceSensor mSenColPxlPos2_DistanceSensor;
    protected final LED mLED1;
    protected final LED mLED2;
    protected final LED mLED3;
    protected final LED mLED4;
    protected final String cfgSrvPxlPos1 = "SrvPxlPos1";
    protected final String cfgSrvPxlPos2 = "SrvPxlPos2";
    protected final String cfgLed1 = "LED1";
    protected final String cfgLed2 = "LED2";
    protected final String cfgLed3 = "LED3";
    protected final String cfgLed4 = "LED4";
    protected final String cfgSenColPxlPos1 = "SenColPxlPos1";
    protected final String cfgSenColPxlPos2 = "SenColPxlPos2";
    protected double cCasBackToFront = 0.2;
    protected double cCasFrontToBack = 0.8;
    protected double cPixelPos1Dist = 1;
    protected double cPixelPos2Dist = 10;
    protected HydraOpMode mOp;
    public HydraPixelPalace(HydraOpMode op) {
        mOp = op;
        mSrvPxlPos1 = mOp.mHardwareMap.get(Servo.class, cfgSrvPxlPos1);
        mSrvPxlPos2 = mOp.mHardwareMap.get(Servo.class, cfgSrvPxlPos2);
        ColorSensor mSenColPxlPos1 = mOp.mHardwareMap.get(ColorSensor.class, cfgSenColPxlPos1);
        mSenColPxlPos1_DistanceSensor = mOp.mHardwareMap.get(DistanceSensor.class, cfgSenColPxlPos1);
        ColorSensor mSenColPxlPos2 = mOp.mHardwareMap.get(ColorSensor.class, cfgSenColPxlPos2);
        mSenColPxlPos2_DistanceSensor = mOp.mHardwareMap.get(DistanceSensor.class, cfgSenColPxlPos2);
        mLED1 = mOp.mHardwareMap.get(LED.class, cfgLed1);
        mLED2 = mOp.mHardwareMap.get(LED.class, cfgLed2);
        mLED3 = mOp.mHardwareMap.get(LED.class, cfgLed3);
        mLED4 = mOp.mHardwareMap.get(LED.class, cfgLed4);
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
                direction = cCasBackToFront;
                break;
            case PixelPalaceFrontToBack:
                direction = cCasFrontToBack;
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
        detPixelPos1 = distPixelPos1 < cPixelPos1Dist;
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
        detPixelPos2 = distPixelPos2 < cPixelPos2Dist;
        //telemetry.addData("PixelPos2Dist", distPixelPos2);
        //telemetry.addData("Pixel2Detect", detPixelPos2);
        return detPixelPos2;
    }
}
