package org.firstinspires.ftc.teamcode.util;

import com.acmerobotics.roadrunner.Rotation2d;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

public final class BNO055Wrapper {
    private final BNO055IMU bno;

    public BNO055Wrapper(BNO055IMU bno) {
        this.bno = bno;
    }

    public Rotation2d getHeading() {
        return Rotation2d.exp(bno.getAngularOrientation()
                .toAngleUnit(AngleUnit.RADIANS)
                .toAxesOrder(AxesOrder.ZYX)
                .firstAngle);
    }

    public double getHeadingVelocity() {
        return getAngularVelocity().zRotationRate;
    }

    public AngularVelocity getAngularVelocity() {
        return bno.getAngularVelocity().toAngleUnit(AngleUnit.RADIANS);
    }

    /**
     * A direction for an axis to be remapped to
     */
    public enum AxisDirection {
        POS_X, NEG_X, POS_Y, NEG_Y, POS_Z, NEG_Z
    }

    /**
     * IMU axes signs in the order XYZ (after remapping).
     */
    public enum AxesSigns {
        PPP(0b000),
        PPN(0b001),
        PNP(0b010),
        PNN(0b011),
        NPP(0b100),
        NPN(0b101),
        NNP(0b110),
        NNN(0b111);

        public final int bVal;

        AxesSigns(int bVal) {
            this.bVal = bVal;
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
     * NOTE: Remapping axes can be somewhat confusing. Instead, use {@link #remapZAxis}, if
     * appropriate.
     *
     * @param order axes order
     * @param signs axes signs
     */
    public void swapThenFlipAxes(AxesOrder order, AxesSigns signs) {
        try {
            // the indices correspond with the 2-bit axis encodings specified in the datasheet
            int[] indices = order.indices();
            // AxesSign's values align with the datasheet
            int axisMapSigns = signs.bVal;

            if (indices[0] == indices[1] || indices[0] == indices[2] || indices[1] == indices[2]) {
                throw new RuntimeException("Same axis cannot be included in AxesOrder twice");
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
                throw new RuntimeException("Coordinate system is left-handed");
            }

            // Bit:  7  6 |  5  4  |  3  2  |  1  0  |
            //   reserved | z axis | y axis | x axis |
            int axisMapConfig = indices[2] << 4 | indices[1] << 2 | indices[0];

            // Enter CONFIG mode
            bno.write8(BNO055IMU.Register.OPR_MODE, BNO055IMU.SensorMode.CONFIG.bVal & 0x0F);

            Thread.sleep(100);

            // Write the AXIS_MAP_CONFIG register
            bno.write8(BNO055IMU.Register.AXIS_MAP_CONFIG, axisMapConfig & 0x3F);

            // Write the AXIS_MAP_SIGN register
            bno.write8(BNO055IMU.Register.AXIS_MAP_SIGN, axisMapSigns & 0x07);

            // Switch back to the previous mode
            bno.write8(BNO055IMU.Register.OPR_MODE, bno.getParameters().mode.bVal & 0x0F);

            Thread.sleep(100);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    /**
     * Remaps the IMU coordinate system so that the remapped +Z faces the provided
     * {@link AxisDirection}. See {@link #swapThenFlipAxes} for details about the remapping.
     *
     * @param direction axis direction
     */
    public void remapZAxis(AxisDirection direction) {
        switch (direction) {
            case POS_X:
                swapThenFlipAxes(AxesOrder.ZYX, AxesSigns.NPP);
                break;
            case NEG_X:
                swapThenFlipAxes(AxesOrder.ZYX, AxesSigns.PPN);
                break;
            case POS_Y:
                swapThenFlipAxes(AxesOrder.XZY, AxesSigns.PNP);
                break;
            case NEG_Y:
                swapThenFlipAxes(AxesOrder.XZY, AxesSigns.PPN);
                break;
            case POS_Z:
                swapThenFlipAxes(AxesOrder.XYZ, AxesSigns.PPP);
                break;
            case NEG_Z:
                swapThenFlipAxes(AxesOrder.XYZ, AxesSigns.PNN);
                break;
        }
    }
}
