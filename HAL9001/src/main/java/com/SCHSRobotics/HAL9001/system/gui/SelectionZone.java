package com.SCHSRobotics.HAL9001.system.gui;

import static java.lang.Math.max;

import org.jetbrains.annotations.Contract;
import org.jetbrains.annotations.NotNull;

/**
 * A class that represents the valid locations of the cursor within a menu.
 * <p>
 * Creation Date: 5/17/20
 *
 * @author Cole Savage, Level Up
 * @version 1.1.0
 * @see HALMenu
 * @since 1.1.0
 */
public final class SelectionZone {

    //A boolean matrix specifying the valid locations of the cursor.
    private boolean[][] zoneMatrix;
    //Whether or not the selection zone is of size 0.
    private boolean isZero;
    //The width and height of the selection zone.
    private int width, height;

    /**
     * Simple constructor for SelectionZone.
     *
     * @param width The width of the selection zone.
     * @param height The height of the selection zone.
     */
    public SelectionZone(int width, int height) {
        isZero = width <= 0 && height <= 0;
        if (!isZero) {
            //Sets all values of the matrix within (width, height) to true.
            zoneMatrix = new boolean[height][width];
            for (int i = 0; i < height; i++) {
                for (int j = 0; j < width; j++) {
                    zoneMatrix[i][j] = true;
                }
            }
            this.width = width;
            this.height = height;
        }
        //If the selection zone is zero, set it equal to a 1x1 matrix with false as the only entry.
        else {
            zoneMatrix = new boolean[1][1];
            this.width = 0;
            this.height = 0;
        }
    }

    /**
     * A more complex constructor for SelectionZone. Uses a boolean matrix.
     *
     * @param selectionZoneMatrix The raw boolean matrix specifying valid cursor locations.
     */
    public SelectionZone(@NotNull boolean[][] selectionZoneMatrix) {
        if (selectionZoneMatrix.length > 0 && allRowsGreaterThanZeroLength(selectionZoneMatrix)) {
            isZero = false;
            zoneMatrix = boxify(selectionZoneMatrix);
            width = zoneMatrix[0].length;
            height = zoneMatrix.length;
        } else {
            isZero = true;
            width = 0;
            height = 0;
        }
    }

    /**
     * A more complex constructor for SelectionZone. Uses an integer matrix for ease of entry.
     *
     * @param selectionZoneMatrix An integer matrix specifying which locations are valid cursor locations. (> 0 is true, <= 0 is false).
     */
    public SelectionZone(int[][] selectionZoneMatrix) {
        this(intMatrixToBoolMatrix(selectionZoneMatrix));
    }

    /**
     * Converts an integer array to a boolean array using the rule > 0 = true and <= 0 = false.
     *
     * @param intArray The integer array to convert to a boolean array.
     * @return A boolean array produced by applying the rule > 0 = true and <= 0 = false to the given integer array.
     */
    @NotNull
    @Contract(pure = true)
    private static boolean[] intArrayToBoolArray(@NotNull int[] intArray) {
        boolean[] boolArray = new boolean[intArray.length];

        //If val is > 0 (preferably 1) it's true. if val <= 0 (preferably 0), it's false
        for (int i = 0; i < intArray.length; i++) boolArray[i] = intArray[i] > 0;

        return boolArray;
    }

    /**
     * Converts an integer matrix to a boolean matrix using the rule > 0 = true and <= 0 = false.
     *
     * @param intMatrix The integer matrix to convert to a matrix array.
     * @return A boolean matrix produced by applying the rule > 0 = true and <= 0 = false to the given integer matrix.
     */
    @NotNull
    @Contract(pure = true)
    private static boolean[][] intMatrixToBoolMatrix(@NotNull int[][] intMatrix) {
        boolean[][] boolMatrix = new boolean[intMatrix[0].length][intMatrix.length];
        for (int i = 0; i < intMatrix.length; i++)
            boolMatrix[i] = intArrayToBoolArray(intMatrix[i]);
        return boolMatrix;
    }

    /**
     * Converts a weirdly shaped boolean matrix into a rectangle by inserting "false" in empty spaces.
     *
     * @param nonBoxMatrix The weirdly shaped boolean matrix.
     * @return A rectangular version of the given matrix.
     */
    @NotNull
    private static boolean[][] boxify(@NotNull boolean[][] nonBoxMatrix) {
        int maxRowLength = 0;
        for (boolean[] row : nonBoxMatrix) maxRowLength = max(maxRowLength, row.length);

        boolean[][] boxMatrix = new boolean[nonBoxMatrix.length][maxRowLength];
        for (int i = 0; i < nonBoxMatrix.length; i++) {
            System.arraycopy(nonBoxMatrix[i], 0, boxMatrix[i], 0, nonBoxMatrix[i].length);
        }
        return boxMatrix;
    }

    /**
     * Checks if all rows in a boolean matrix have a nonzero length.
     *
     * @param matrix The matrix to check.
     * @return Whether all rows in the given boolean matrix have a nonzero length.
     */
    @Contract(pure = true)
    private static boolean allRowsGreaterThanZeroLength(@NotNull boolean[][] matrix) {
        boolean greaterThanZeroLength = true;
        for (boolean[] row : matrix) greaterThanZeroLength &= row.length > 0;
        return greaterThanZeroLength;
    }

    /**
     * Gets whether a specified x, y position is a valid cursor position.
     *
     * @param x The cursor x position.
     * @param y The cursor y position.
     * @return Whether the specified x, y position is a valid cursor position.
     */
    public final boolean isValidLocation(int x, int y) {
        if (x >= width || y >= height) return false;
        return zoneMatrix[y][x];
    }

    /**
     * Sets a specific value in the selection zone.
     *
     * @param x     The x coordinate of the value's position.
     * @param y     The y coordinate of the value's position.
     * @param value The value to set at the given position.
     */
    public final void setValue(int x, int y, boolean value) {
        if (x < width && y < height) zoneMatrix[y][x] = value;
    }

    /**
     * Adds a row to the bottom of the selection zone.
     *
     * @param row The row to add to the bottom of the selection zone.
     */
    public final void addRow(boolean[] row) {
        if (isZero) {
            zoneMatrix = boxify(new boolean[][]{row});
            isZero = false;
        } else {
            boolean[][] newZoneMatrixNonBox = new boolean[zoneMatrix.length + 1][zoneMatrix[0].length];
            System.arraycopy(zoneMatrix, 0, newZoneMatrixNonBox, 0, zoneMatrix.length);
            newZoneMatrixNonBox[zoneMatrix.length] = row;
            zoneMatrix = boxify(newZoneMatrixNonBox);
        }
        width = zoneMatrix[0].length;
        height = zoneMatrix.length;
    }

    /**
     * Adds a row to the bottom of the selection zone.
     *
     * @param row The row to add to the bottom of the selection zone (> 0 is true, <= 0 is false).
     */
    public final void addRow(int[] row) {
        addRow(intArrayToBoolArray(row));
    }

    /**
     * Gets whether the selection zone is zero.
     *
     * @return Whether the selection zone is zero.
     */
    public final boolean isZero() {
        return isZero;
    }

    /**
     * Gets the width of the selection zone.
     *
     * @return The width of the selection zone.
     */
    public int getWidth() {
        return width;
    }

    /**
     * Gets the height of the selection zone.
     *
     * @return The height of the selection zone.
     */
    public int getHeight() {
        return height;
    }
}