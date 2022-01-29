package com.SCHSRobotics.HAL9001.system.robot.roadrunner_util;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.jetbrains.annotations.NotNull;

/**
 * Various utility functions for the BNO055 IMU.
 */
public class BNO055IMUUtil {
    /**
     * Remap BNO055 IMU axes and signs. For reference, the default order is {@link AxesOrder#ZYX}.
     * Call after {@link BNO055IMU#initialize(BNO055IMU.Parameters)}. Although this isn't
     * mentioned in the datasheet, the axes order appears to affect the onboard sensor fusion.
     * <p>
     * Adapted from <a href="https://ftcforum.usfirst.org/forum/ftc-technology/53812-mounting-the-revhub-vertically?p=56587#post56587">this post</a>.
     *
     * @param imu   IMU
     * @param order axes order
     * @param signs axes signs
     */
    public static void remapAxes(@NotNull BNO055IMU imu, @NotNull AxesOrder order, @NotNull AxesSigns signs) {
        try {
            // the indices correspond with the 2-bit encodings specified in the datasheet
            int[] indices = order.indices();
            int axisMapConfig = 0;
            axisMapConfig |= (indices[0] << 4);
            axisMapConfig |= (indices[1] << 2);
            axisMapConfig |= (indices[2]);

            // the BNO055 driver flips the first orientation vector so we also flip here
            int axisMapSign = signs.bVal ^ (0b100 >> indices[0]);

            // Enter CONFIG mode
            imu.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

            Thread.sleep(100);

            // Write the AXIS_MAP_CONFIG register
            imu.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, axisMapConfig & 0x3F);

            // Write the AXIS_MAP_SIGN register
            imu.write8(BNO055IMU.Register.AXIS_MAP_SIGN, axisMapSign & 0x07);

            // Switch back to the previous mode
            imu.write8(BNO055IMU.Register.OPR_MODE, imu.getParameters().mode.bVal & 0x0F);

            Thread.sleep(100);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}