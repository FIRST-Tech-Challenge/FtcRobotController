package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

/**
 * Various utility functions for the BNO055 IMU.
 */
public class BNO055IMUUtil {
    /**
     * Error for attempting an illegal remapping (lhs or multiple same axes)
     */
    public static class InvalidAxisRemapException extends RuntimeException {
        public InvalidAxisRemapException(String detailMessage) {
            super(detailMessage);
        }
    }

    /**
     * Remap BNO055 IMU axes and signs. For reference, the default order is {@link AxesOrder#XYZ}.
     * Call after {@link BNO055IMU#initialize(BNO055IMU.Parameters)}. Although this isn't
     * mentioned in the datasheet, the axes order appears to affect the onboard sensor fusion.
     *
     * Adapted from <a href="https://ftcforum.firstinspires.org/forum/ftc-technology/53812-mounting-the-revhub-vertically?p=56587#post56587">this post</a>.
     * Reference the <a href="https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf">BNO055 Datasheet</a> for details.
     *
     * NOTE: Remapping axes can be somewhat confusing. Instead, use {@link #remapZAxis}, if appropriate.
     *
     * @param imu IMU
     * @param order axes order
     * @param signs axes signs
     */
    public static void swapThenFlipAxes(BNO055IMU imu, AxesOrder order, AxesSigns signs) {
        try {
            // the indices correspond with the 2-bit axis encodings specified in the datasheet
            int[] indices = order.indices();
            // AxesSign's values align with the datasheet
            int axisMapSigns = signs.bVal;

            if (indices[0] == indices[1] || indices[0] == indices[2] || indices[1] == indices[2]) {
                throw new InvalidAxisRemapException("Same axis cannot be included in AxesOrder twice");
            }

            // ensure right-handed coordinate system
            boolean isXSwapped = indices[0] != 0;
            boolean isYSwapped = indices[1] != 1;
            boolean isZSwapped = indices[2] != 2;
            boolean areTwoAxesSwapped = (isXSwapped || isYSwapped || isZSwapped)
                    && (!isXSwapped || !isYSwapped || !isZSwapped);
            boolean oddNumOfFlips = (((axisMapSigns >> 2) ^ (axisMapSigns >> 1) ^ axisMapSigns) & 1) == 1;
            // != functions as xor
            if (areTwoAxesSwapped != oddNumOfFlips) {
                throw new InvalidAxisRemapException("Coordinate system is left-handed");
            }

            // Bit:  7  6 |  5  4  |  3  2  |  1  0  |
            //   reserved | z axis | y axis | x axis |
            int axisMapConfig = indices[2] << 4 | indices[1] << 2 | indices[0];

            // Enter CONFIG mode
            imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

            Thread.sleep(100);

            // Write the AXIS_MAP_CONFIG register
            imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, axisMapConfig & 0x3F);

            // Write the AXIS_MAP_SIGN register
            imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, axisMapSigns & 0x07);

            // Switch back to the previous mode
            imu.write8(BNO055IMU.Register.OPR_MODE, imu.getParameters().mode.bVal & 0x0F);

            Thread.sleep(100);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Remaps the IMU coordinate system so that the remapped +Z faces the provided
     * {@link AxisDirection}. See {@link #swapThenFlipAxes} for details about the remapping.
     *
     * @param imu IMU
     * @param direction axis direction
     */
    public static void remapZAxis(BNO055IMU imu, AxisDirection direction) {
        switch (direction) {
            case POS_X:
                swapThenFlipAxes(imu, AxesOrder.ZYX, AxesSigns.NPP);
                break;
            case NEG_X:
                swapThenFlipAxes(imu, AxesOrder.ZYX, AxesSigns.PPN);
                break;
            case POS_Y:
                swapThenFlipAxes(imu, AxesOrder.XZY, AxesSigns.PNP);
                break;
            case NEG_Y:
                swapThenFlipAxes(imu, AxesOrder.XZY, AxesSigns.PPN);
                break;
            case POS_Z:
                swapThenFlipAxes(imu, AxesOrder.XYZ, AxesSigns.PPP);
                break;
            case NEG_Z:
                swapThenFlipAxes(imu, AxesOrder.XYZ, AxesSigns.PNN);
                break;
        }
    }

    /**
     * Now deprecated due to unintuitive parameter order.
     * Use {@link #swapThenFlipAxes} or {@link #remapZAxis} instead.
     *
     * @param imu IMU
     * @param order axes order
     * @param signs axes signs
     */
    @Deprecated
    public static void remapAxes(BNO055IMU imu, AxesOrder order, AxesSigns signs) {
        AxesOrder adjustedAxesOrder = order.reverse();
        int[] indices = order.indices();
        int axisSignValue = signs.bVal ^ (0b100 >> indices[0]);
        AxesSigns adjustedAxesSigns = AxesSigns.fromBinaryValue(axisSignValue);

        swapThenFlipAxes(imu, adjustedAxesOrder, adjustedAxesSigns);
    }
}
