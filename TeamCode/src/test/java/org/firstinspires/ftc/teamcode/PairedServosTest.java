package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import org.horizon36596.simloop.fakehardware.FakeDigitalChannel;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.horizon36596.simloop.fakehardware.FakeServo;
import org.junit.Test;

import static org.junit.Assert.*;

/** The geared turret rotation servos must never receive different directions, ranges or positions. */
public class PairedServosTest {
    @Test public void configuresOneDirectionAndRangeAndWritesBothTogether() {
        FakeServo a = new FakeServo(), b = new FakeServo();
        b.setDirection(Servo.Direction.REVERSE); // A stale per-servo setting must be overwritten.
        PairedServos pair = new PairedServos("pair", a, b, Servo.Direction.FORWARD, 500, 2500);
        assertEquals(Servo.Direction.FORWARD, a.getDirection());
        assertEquals(a.getDirection(), b.getDirection());
        assertRange(a, 500, 2500); assertRange(b, 500, 2500);
        for (double p : new double[]{.5, .1, .9, .37}) {
            pair.setPosition(p);
            assertEquals(p, a.getPosition(), 0); assertEquals(p, b.getPosition(), 0);
        }
        pair.setPosition(Double.NaN);
        assertEquals(.37, a.getPosition(), 0); assertEquals(.37, b.getPosition(), 0);
    }

    @Test public void detectsDivergedDirectionOrRange() {
        FakeServo a = new FakeServo(), b = new FakeServo();
        PairedServos pair = new PairedServos("pair", a, b, Servo.Direction.FORWARD, 500, 2500);
        b.setDirection(Servo.Direction.REVERSE);
        assertThrows(IllegalStateException.class, pair::verifyMatched);
        b.setDirection(Servo.Direction.FORWARD);
        b.setPwmRange(new PwmControl.PwmRange(600, 2400));
        assertThrows(IllegalStateException.class, pair::verifyMatched);
    }

    @Test public void rejectsSameServoTwice() {
        FakeServo a = new FakeServo();
        assertThrows(IllegalArgumentException.class,
                () -> new PairedServos("pair", a, a, Servo.Direction.FORWARD, 500, 2500));
    }

    @Test public void turretDrivesBothRotationServosIdentically() {
        FakeHardwareMap map = new FakeHardwareMap();
        map.register("intake_motor", new FakeMotor(2590, 10, 28));
        map.register("transfer_motor_1", new FakeMotor(2800, 10, 28));
        map.register("transfer_motor_2", new FakeMotor(2590, 10, 28));
        map.register("turret_motor", new FakeMotor(2590, 10, 28));
        map.register("distance_sensor", new FakeDigitalChannel());
        FakeServo yaw1 = new FakeServo(), yaw2 = new FakeServo();
        yaw2.setDirection(Servo.Direction.REVERSE);
        map.register("turret_servo_1", yaw1); map.register("turret_servo_2", yaw2);
        map.register("turret_pitch", new FakeServo());
        TurretSubsystem turret = new TurretSubsystem(map, false, () -> 0);
        assertEquals(TurretSubsystem.ROTATION_SERVO_DIRECTION, yaw1.getDirection());
        assertEquals(yaw1.getDirection(), yaw2.getDirection());
        assertRange(yaw1, AxonMaxPlus.MIN_PULSE_US, AxonMaxPlus.MAX_PULSE_US);
        assertRange(yaw2, AxonMaxPlus.MIN_PULSE_US, AxonMaxPlus.MAX_PULSE_US);
        assertEquals(.5, yaw1.getPosition(), 0); assertEquals(.5, yaw2.getPosition(), 0);
        for (double p : new double[]{.2, .8, -3, 4}) {
            turret.setRawRotationServoPosition(p);
            assertEquals(turret.getLastCommandedRotationServoPos(), yaw1.getPosition(), 0);
            assertEquals(yaw1.getPosition(), yaw2.getPosition(), 0);
        }
    }

    private static void assertRange(FakeServo servo, double lower, double upper) {
        PwmControl.PwmRange range = servo.getPwmRange();
        assertEquals(lower, range.usPulseLower, 0); assertEquals(upper, range.usPulseUpper, 0);
    }
}
