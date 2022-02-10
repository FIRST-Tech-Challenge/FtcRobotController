package org.firstinspires.ftc.teamcode.src.utills.enums;

import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * An enum returned for position in camera view
 */
public enum BarcodePositions {
    NotSeen,
    Right,
    Center,
    Left;

    private static final double[] RightPositionCoordinates = new double[]{610, 225};
    private static final double[] CenterPositionCoordinates = new double[]{399, 228};
    private static final double[] LeftPositionCoordinates = new double[]{190, 240};

    private static final double[] NotSeenPositionCoordinate = new double[]{Double.MIN_VALUE, Double.MIN_VALUE};
    //These are at minimum values so that the distance getting algorithm will never return a position coordinate for NotSeenPositionCoordinate, yet I don't have to check to see if it is passed back
    private static final BarcodePositions[] barcodePositions = BarcodePositions.values();
    private static final double[][] positionValuesOfEachItem;

    static {
        positionValuesOfEachItem = new double[BarcodePositions.barcodePositions.length][2];
        for (int x = 0; x < BarcodePositions.barcodePositions.length; x++) {
            positionValuesOfEachItem[x] = getPositionFromItem(BarcodePositions.barcodePositions[x]);
        }
    }

    private static double[] getPositionFromItem(BarcodePositions item) {
        switch (item) {
            case Right:
                return RightPositionCoordinates;
            case Left:
                return LeftPositionCoordinates;
            case Center:
                return CenterPositionCoordinates;
            case NotSeen:
                return NotSeenPositionCoordinate;
            default:
                return null;
        }
    }

    public static BarcodePositions getRecognitionLocation(Recognition recognition) {
        if (recognition == null) {
            return NotSeen;
        }

        final double[] itemCenterCoordinate = new double[2];
        itemCenterCoordinate[0] = (recognition.getLeft() + recognition.getRight()) / 2.0;
        itemCenterCoordinate[1] = (recognition.getTop() + recognition.getBottom()) / 2.0;


        final int numberOfBarcodePositions = BarcodePositions.barcodePositions.length;

        final double[] differences = new double[numberOfBarcodePositions];

        for (int x = 0; x < numberOfBarcodePositions; x++) {
            differences[x] = getDistance(itemCenterCoordinate, positionValuesOfEachItem[x]);
        }


        //These variables are guaranteed to be written to with the first iteration of the loop, yet must be initialized for the compiler
        double smallestDistance = Double.MAX_VALUE;
        int indexOfSmallestValue = -1;

        for (int i = 0; i < numberOfBarcodePositions; i++) {
            if (differences[i] < smallestDistance) {
                smallestDistance = differences[i];
                indexOfSmallestValue = i;
            }
        }

        return BarcodePositions.barcodePositions[indexOfSmallestValue];

    }

    private static double getDistance(double[] point1, double[] point2) {
        return Math.sqrt((point2[1] - point1[1]) * (point2[1] - point1[1]) + (point2[0] - point1[0]) * (point2[0] - point1[0]));
    }


}

