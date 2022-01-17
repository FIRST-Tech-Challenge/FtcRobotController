package org.firstinspires.ftc.teamcode.src.utills.enums;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.teamcode.src.robotAttachments.subsystems.ContinuousIntake;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;

/**
 * A Enum for each object the bucket can pick up
 */
public enum FreightFrenzyGameObject {
    BALL,
    CUBESMOOTH,
    CUBEWAFFLE,
    DUCK,
    EMPTY;

    /**
     * The Key is the game object, the value is what LED pattern it should corespond to
     */
    protected static final HashMap<FreightFrenzyGameObject, RevBlinkinLedDriver.BlinkinPattern> RevColorOfObj = new HashMap<FreightFrenzyGameObject, RevBlinkinLedDriver.BlinkinPattern>() {{
        put(FreightFrenzyGameObject.BALL, RevBlinkinLedDriver.BlinkinPattern.WHITE);
        put(FreightFrenzyGameObject.CUBESMOOTH, RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        put(FreightFrenzyGameObject.CUBEWAFFLE, RevBlinkinLedDriver.BlinkinPattern.ORANGE);
        put(FreightFrenzyGameObject.DUCK, RevBlinkinLedDriver.BlinkinPattern.GREEN);
        put(FreightFrenzyGameObject.EMPTY, null);
    }};
    /**
     * A list of every possible enum value
     */
    private static final ArrayList<FreightFrenzyGameObject> gameObjectList = new ArrayList<>(Arrays.asList(FreightFrenzyGameObject.values()));
    /**
     * The Key is the game object, the value is the RGB value of what the sensor sees
     */
    private static final HashMap<FreightFrenzyGameObject, double[]> RGBOfObj = new HashMap<FreightFrenzyGameObject, double[]>() {{
        put(FreightFrenzyGameObject.BALL, new double[]{111, 111, 97});
        put(FreightFrenzyGameObject.CUBESMOOTH, new double[]{41, 25, 15});
        put(FreightFrenzyGameObject.CUBEWAFFLE, new double[]{26, 15, 10});
        put(FreightFrenzyGameObject.DUCK, new double[]{14, 12, 7});
        put(FreightFrenzyGameObject.EMPTY, new double[]{5, 6, 5});
    }};

    /**
     * Determines what game object best matches the color pattern provided
     *
     * @param RGB The color pattern in the form of RGB
     * @return Returns what game object the RGB best matches
     */
    public static FreightFrenzyGameObject identify(double[] RGB) {
        int numberOfGameElements = FreightFrenzyGameObject.gameObjectList.size();
        double[][] originalRGB = new double[numberOfGameElements][3];
        double[] differences = new double[numberOfGameElements];
        for (int x = 0; x < numberOfGameElements; x++) {
            originalRGB[x] = FreightFrenzyGameObject.RGBOfObj.get((FreightFrenzyGameObject.gameObjectList.get(x)));
        }
        for (int x = 0; x < numberOfGameElements; x++) {
            assert originalRGB[x] != null;
            differences[x] = ContinuousIntake.getDifferenceOfColor(RGB, originalRGB[x]);
        }
        double smallestValue = Double.MAX_VALUE;
        int index = -1;

        for (int i = 0; i < numberOfGameElements; i++) {
            if (differences[i] < smallestValue) {
                smallestValue = differences[i];
                index = i;
            }
        }
        return FreightFrenzyGameObject.gameObjectList.get(index);


    }

}
