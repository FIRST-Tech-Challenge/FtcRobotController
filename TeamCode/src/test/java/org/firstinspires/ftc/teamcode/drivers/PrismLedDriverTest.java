package org.firstinspires.ftc.teamcode.drivers;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.AdditionalMatchers.aryEq;
import static org.mockito.ArgumentMatchers.eq;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;

/** The Prism driver with a pretend I2C port: exactly which bytes go out. */
@DisplayName("PrismLedDriver")
class PrismLedDriverTest {

    @Test
    @DisplayName("is built on I2C address 0x38")
    void address() {
        I2cDeviceSynchSimple port = mock(I2cDeviceSynchSimple.class);
        new PrismLedDriver(port, true);
        ArgumentCaptor<I2cAddr> address = ArgumentCaptor.forClass(I2cAddr.class);
        verify(port).setI2cAddress(address.capture());
        assertEquals(0x38, address.getValue().get7Bit());
    }

    @Test
    @DisplayName("a layer write is the layer's register, the sub-register, then the data")
    void layerWrite() {
        I2cDeviceSynchSimple port = mock(I2cDeviceSynchSimple.class);
        PrismLedDriver prism = new PrismLedDriver(port, true);
        prism.WriteLayer(0, 0x04, new byte[] {(byte) 255, 0, 0});
        verify(port).write(eq(0x08), aryEq(new byte[] {0x04, (byte) 255, 0, 0}));
        prism.WriteLayer(3, 0x00, new byte[] {1});
        verify(port).write(eq(0x0B), aryEq(new byte[] {0x00, 1}));
    }

    @Test
    @DisplayName("SolidColor on a layer: animation 1, brightness, start, stop, color, in that order")
    void solidColor() {
        I2cDeviceSynchSimple port = mock(I2cDeviceSynchSimple.class);
        PrismLedDriver prism = new PrismLedDriver(port, true);
        prism.SolidColor(2, 2, 3, 50, 255, 96, 0);
        org.mockito.InOrder order = org.mockito.Mockito.inOrder(port);
        order.verify(port).write(eq(0x0A), aryEq(new byte[] {0x00, 1}));
        order.verify(port).write(eq(0x0A), aryEq(new byte[] {0x01, 50}));
        order.verify(port).write(eq(0x0A), aryEq(new byte[] {0x02, 2}));
        order.verify(port).write(eq(0x0A), aryEq(new byte[] {0x03, 3}));
        order.verify(port).write(eq(0x0A), aryEq(new byte[] {0x04, (byte) 255, 96, 0}));
    }

    @Test
    @DisplayName("ClearLayer: animation 0 on that layer")
    void clearLayer() {
        I2cDeviceSynchSimple port = mock(I2cDeviceSynchSimple.class);
        new PrismLedDriver(port, true).ClearLayer(9);
        verify(port).write(eq(0x11), aryEq(new byte[] {0x00, 0}));
    }

    @Test
    @DisplayName("reads the device id as 0 to 255 and the status as four bytes")
    void reads() {
        I2cDeviceSynchSimple port = mock(I2cDeviceSynchSimple.class);
        when(port.read8(PrismLedDriver.REG_DEVICE_ID)).thenReturn((byte) 0xA5);
        when(port.read(PrismLedDriver.REG_STATUS, 4)).thenReturn(new byte[] {12, 60, 0, 0});
        PrismLedDriver prism = new PrismLedDriver(port, true);
        assertEquals(0xA5, prism.ReadDeviceId());
        assertEquals(4, prism.ReadStatus().length);
    }
}
