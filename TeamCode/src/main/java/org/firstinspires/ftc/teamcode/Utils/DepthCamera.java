package org.firstinspires.ftc.teamcode.Utils;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@Config
public class DepthCamera {
    public static int FRONT_CAMERA = 1;
    public static int BACK_CAMERA = 2;

    public static int ADVANCED_RANGE = 500;
    public static int ADVANCED_MAX_DURATION_MILLIS = 400;

    public static int FILTER_DURATION = 3;
    public static int MAX_ALLOWED_VALUE = 10000;

    public final static int MAX_X = 1280;   //Low Res: 848. High Res: 1280.
    public final static int MAX_Y = 720;    //Low Res: 480. High Res: 720.
    public final static double HORIZONTAL_FOV_RAD = 1.51843645;
    public final static double VERTICAL_FOV_RAD = 1.012291;

    public static int FRONT_TARGET_X = MAX_X / 2;
    public static int FRONT_TARGET_Y = 240;
    public static int FRONT_TARGET_D = 2800;

    public static int BACK_TARGET_X = MAX_X / 2;
    public static int BACK_TARGET_Y = 240;
    public static int BACK_TARGET_D = 1800;
    public static double BACK_CAMERA_TILT_RAD = 0.680678;

    public static int HIGH_JUNCTION_HEIGHT = 500;
    public static int MID_JUNCTION_HEIGHT = 300;

    public static boolean USING_FILTER = true;

    public final static String frontCameraId = "128422272377";
    public final static String backCameraId = "126122270141";

    private static Long frontCameraHandle = null;
    private static Long backCameraHandle = null;

    private final MedianFilter dFilter = new MedianFilter(FILTER_DURATION);
    private final MedianFilter rFilter = new MedianFilter(FILTER_DURATION);

    private long cameraHandle;
    private boolean isFrontCamera;
    private int lastXIndex = -1;
    private long lastXIndexTime = 0;

    public DepthCamera(OpMode opMode, int cameraId) {
        opMode.msStuckDetectStop = 5000;
        System.loadLibrary("t265");

        if (cameraId == DepthCamera.FRONT_CAMERA) {
            if (frontCameraHandle == null) {
                frontCameraHandle = init(frontCameraId);
            }
            cameraHandle = frontCameraHandle;

            isFrontCamera = true;
        } else {
            if (backCameraHandle == null) {
                backCameraHandle = init(backCameraId);
            }
            cameraHandle = backCameraHandle;

            isFrontCamera = false;
        }
    }

    public void turnOn() {
        startDepth();
    }

    public boolean turnOff() {
        try {
            stop(cameraHandle);
            return true;
        } catch (Exception ignored) {
            return false;
        }
    }

    public void startDepth() {
        startDepth(cameraHandle);
    }

    public void startVideo() {
        startVideo(cameraHandle);
    }

    public char[][] getDepthData() {
        return getDepthData(cameraHandle);
    }

    public Pose2d getDepthError() {
        int targetX, targetY, targetD;

        if (isFrontCamera) {
            targetX = FRONT_TARGET_X;
            targetY = FRONT_TARGET_Y;
            targetD = FRONT_TARGET_D;
        } else {
            targetX = BACK_TARGET_X;
            targetY = BACK_TARGET_Y;
            targetD = BACK_TARGET_D;
        }

        int xIndex = targetX;
        int yIndex = targetY;

        //Get Raw depth data from the camera.
        char[][] depthData = getDepthData();

        for (; yIndex < MAX_Y - 1; yIndex++) {
            xIndex = getRowMinIndex(depthData[yIndex]);

            if (depthData[yIndex][xIndex] != 0 && depthData[yIndex][xIndex] < MAX_ALLOWED_VALUE)
                break;
        }

        double d = depthData[yIndex][xIndex];
        double r = HORIZONTAL_FOV_RAD * (targetX - xIndex) / DepthCamera.MAX_X;

        if (USING_FILTER) {
            //Filter the values.
            d = dFilter.calculate(d);
            r = rFilter.calculate(r);
        }

//        Log.d("Depth Error", "" + (d - targetD));

        return new Pose2d(
                d - targetD,
                toRelativeDistance(r, d),
                r
        );
    }

    public Pose2d getDepthError(int targetD) {
        int targetX, targetY;

        if (isFrontCamera) {
            targetX = FRONT_TARGET_X;
            targetY = FRONT_TARGET_Y;
        } else {
            targetX = BACK_TARGET_X;
            targetY = BACK_TARGET_Y;
        }

        int xIndex = targetX;
        int yIndex = targetY;

        //Get Raw depth data from the camera.
        char[][] depthData = getDepthData();

        for (; yIndex < MAX_Y - 1; yIndex++) {
            xIndex = getRowMinIndex(depthData[yIndex]);

            if (depthData[yIndex][xIndex] != 0 && depthData[yIndex][xIndex] < MAX_ALLOWED_VALUE)
                break;
        }

        double d = depthData[yIndex][xIndex];
        double r = HORIZONTAL_FOV_RAD * (targetX - xIndex) / DepthCamera.MAX_X;

        if (USING_FILTER) {
            //Filter the values.
            d = dFilter.calculate(d);
            r = rFilter.calculate(r);
        }

//        Log.d("Depth Error", "" + (d - targetD));

        return new Pose2d(
                d - targetD,
                toRelativeDistance(r, d),
                r
        );
    }

    public Pose2d getDepthErrorAdvanced(int targetDepth) {
        long currentTimeMillis = System.currentTimeMillis();
        int targetX, targetY, targetD;

        if (isFrontCamera) {
            targetX = FRONT_TARGET_X;
            targetY = FRONT_TARGET_Y;
        } else {
            targetX = BACK_TARGET_X;
            targetY = BACK_TARGET_Y;
        }

        targetD = targetDepth;

        int xIndex = targetX;
        int yIndex = targetY;

        //Get Raw depth data from the camera.
        char[][] depthData = getDepthData();

        if (currentTimeMillis - lastXIndexTime > ADVANCED_MAX_DURATION_MILLIS) {
            for (; yIndex < MAX_Y - 1; yIndex++) {
                xIndex = getRowMinIndex(depthData[yIndex]);

                if (depthData[yIndex][xIndex] != 0 && depthData[yIndex][xIndex] < MAX_ALLOWED_VALUE)
                    break;
            }
        } else {
            for (; yIndex < MAX_Y - 1; yIndex++) {
                xIndex = getRowMinIndex(depthData[yIndex], lastXIndex);

                if (depthData[yIndex][xIndex] != 0 && depthData[yIndex][xIndex] < MAX_ALLOWED_VALUE)
                    break;
            }
        }

        double d = depthData[yIndex][xIndex];
        double r = HORIZONTAL_FOV_RAD * (targetX - xIndex) / DepthCamera.MAX_X;

        if (USING_FILTER) {
            //Filter the values.
            d = dFilter.calculate(d);
            r = rFilter.calculate(r);
        }

//        Log.d("index_difference", "" + (xIndex - lastXIndex));

        lastXIndexTime = currentTimeMillis;
        lastXIndex = xIndex;

        return new Pose2d(
                d - targetD,
                toRelativeDistance(r, d),
                r
        );
    }

    public Pose2d getRawDepthDataAdvanced() {
        long currentTimeMillis = System.currentTimeMillis();
        int targetX, targetY, targetD;

        if (isFrontCamera) {
            targetX = FRONT_TARGET_X;
            targetY = FRONT_TARGET_Y;
            targetD = FRONT_TARGET_D;
        } else {
            targetX = BACK_TARGET_X;
            targetY = BACK_TARGET_Y;
            targetD = BACK_TARGET_D;
        }

        int xIndex = targetX;
        int yIndex = targetY;

        //Get Raw depth data from the camera.
        char[][] depthData = getDepthData();

        if (currentTimeMillis - lastXIndexTime > ADVANCED_MAX_DURATION_MILLIS) {
            for (; yIndex < MAX_Y - 1; yIndex++) {
                xIndex = getRowMinIndex(depthData[yIndex]);

                if (depthData[yIndex][xIndex] != 0 && depthData[yIndex][xIndex] < MAX_ALLOWED_VALUE)
                    break;
            }
        } else {
            for (; yIndex < MAX_Y - 1; yIndex++) {
                xIndex = getRowMinIndex(depthData[yIndex], lastXIndex);

                if (depthData[yIndex][xIndex] != 0 && depthData[yIndex][xIndex] < MAX_ALLOWED_VALUE)
                    break;
            }
        }


        double d = depthData[yIndex][xIndex];
        double r = HORIZONTAL_FOV_RAD * (targetX - xIndex) / DepthCamera.MAX_X;

        if (USING_FILTER) {
            //Filter the values.
            d = dFilter.calculate(d);
            r = rFilter.calculate(r);
        }

//        Log.d("Depth Error", "" + (d - targetD));

        lastXIndexTime = currentTimeMillis;
        lastXIndex = xIndex;

        return new Pose2d(
                d - targetD,
                toRelativeDistance(r, d),
                r
        );
    }

    @Deprecated
    public Pose2d getRawDepthErrorRotated() {
        int targetX, targetY;

        targetX = MAX_X / 2;
        targetY = MAX_Y / 2;

        int xIndex = targetX;
        int yIndex = targetY;

        //Get Raw depth data from the camera.
        char[][] depthData = getDepthData();

        for (; xIndex < MAX_X - 1; xIndex++) {
            yIndex = getColumnMinIndex(depthData, xIndex);

            if (depthData[yIndex][xIndex] != 0 && depthData[yIndex][xIndex] < MAX_ALLOWED_VALUE)
                break;
        }

        double d = depthData[yIndex][xIndex];
        double r = HORIZONTAL_FOV_RAD * (targetY - yIndex) / DepthCamera.MAX_Y;

        if (USING_FILTER) {
            //Filter the values.
            d = dFilter.calculate(d);
            r = rFilter.calculate(r);
        }

//        Log.d("Indexes", String.format("x: %d, y: %d, d: %c", xIndex, yIndex, depthData[yIndex][xIndex]));

        return new Pose2d(
                d,
                toRelativeDistance(r, d),
                r
        );
    }

    public Pose2d getRawDepthData() {
        int targetX, targetY;

        if (isFrontCamera) {
            targetX = FRONT_TARGET_X;
            targetY = FRONT_TARGET_Y;
        } else {
            targetX = BACK_TARGET_X;
            targetY = BACK_TARGET_Y;
        }

        int xIndex = targetX;
        int yIndex = targetY;

        //Get Raw depth data from the camera.
        char[][] depthData = getDepthData();

        for (; yIndex < MAX_Y - 1; yIndex++) {
            xIndex = getRowMinIndex(depthData[yIndex]);

            if (depthData[yIndex][xIndex] != 0 && depthData[yIndex][xIndex] < MAX_ALLOWED_VALUE)
                break;
        }

        double d = depthData[yIndex][xIndex];
        double r = HORIZONTAL_FOV_RAD * (targetX - xIndex) / DepthCamera.MAX_X;

        if (USING_FILTER) {
            //Filter the values.
            d = dFilter.calculate(d);
            r = rFilter.calculate(r);
        }

        return new Pose2d(
                d,
                toRelativeDistance(r, d),
                r
        );
    }

    public char getArrayMinValue(char[] array) {
        char min = Character.MAX_VALUE;
        for (char i : array) {
            if (i == 0)
                continue;

            if (i < min)
                min = i;
        }
        return min;
    }

    public int getRowMinIndex(char[] array) {
        int minIndex = 0;
        int i = 0;

        //Find the first non-zero index.
        for (; i < array.length; i++) {
            if (array[i] != 0) {
                minIndex = i;
                break;
            }
        }

        //Find the minimum value.
        for (; i < array.length; i++) {
            if (array[i] == 0)
                continue;

            if (array[i] < array[minIndex]) {
                minIndex = i;
            }
        }

        return minIndex;
    }

    public int getRowMinIndex(char[] array, int midIndex) {
        int rangeStart = midIndex - ADVANCED_RANGE / 2;
        int rangeEnd = midIndex + ADVANCED_RANGE / 2;

        rangeStart = KodiakMath.between(rangeStart, 0, MAX_X);
        rangeEnd = KodiakMath.between(rangeEnd, 0, MAX_X);

        int minIndex = rangeStart;
        int i = rangeStart;

        //Find the first non-zero index.
        for (; i < rangeEnd; i++) {
            if (array[i] != 0) {
                minIndex = i;
                break;
            }
        }

        //Find the minimum value.
        for (; i < rangeEnd; i++) {
            if (array[i] == 0)
                continue;

            if (array[i] < array[minIndex]) {
                minIndex = i;
            }
        }

        return minIndex;
    }

    public int getColumnMinIndex(char[][] depthData, int columnNumber) {
        int minIndex = 0;
        int i = 0;

        //Find the first non-zero index.
        for (; i < depthData.length; i++) {
            if (depthData[i][columnNumber] != 0) {
                minIndex = i;
                break;
            }
        }

        //Find the minimum value.
        for (; i < depthData.length; i++) {
            if (depthData[i][columnNumber] == 0)
                continue;

            if (depthData[i][columnNumber] < depthData[minIndex][columnNumber]) {
                minIndex = i;
            }
        }

        return minIndex;
    }

    public int getBackCameraTopRowNumber(char[][] depthData, int junctionId) {
        //Get horizontal row.
        int horizontalY = (int) Math.round(MAX_Y / 2.0 - MAX_Y * BACK_CAMERA_TILT_RAD / VERTICAL_FOV_RAD);
        double horizontalMinDistance = getRowMinIndex(depthData[horizontalY]);

        //Calculate targetRow.
        int junctionHeight;
        if (junctionId == 0) {
            junctionHeight = HIGH_JUNCTION_HEIGHT;
        } else {
            junctionHeight = MID_JUNCTION_HEIGHT;
        }

        double theta = Math.atan(junctionHeight / horizontalMinDistance);

        int rowNumber = (int) Math.round(MAX_Y / 2.0 + (theta - BACK_CAMERA_TILT_RAD) * MAX_Y / VERTICAL_FOV_RAD);

        return rowNumber;
    }

    private double toRelativeDistance(double rRadian, double distance) {
        return distance * Math.tan(rRadian);
    }

    public byte[] getImageData() {
        return getImageData(cameraHandle);
    }

    private native long init(long id);

    private native long init(String id);

    private native void startDepth(long handle);

    private native void startVideo(long handle);

    public native char[][] getDepthData(long handle);

    private native byte[] getImageData(long handle);

    private native void stop(long handle);
}
