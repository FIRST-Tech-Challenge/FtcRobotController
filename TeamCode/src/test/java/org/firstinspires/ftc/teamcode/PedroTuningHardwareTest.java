package org.firstinspires.ftc.teamcode;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.drivetrain.Drivetrain;
import com.pedropathing.localization.Localizer;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.Tuner;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.pedro.Tuning;
import org.horizon36596.simloop.fakehardware.FakeGoBildaPinpoint;
import org.horizon36596.simloop.fakehardware.FakeHardwareMap;
import org.horizon36596.simloop.fakehardware.FakeMotor;
import org.junit.Before;
import org.junit.Test;

import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.List;

import static org.junit.Assert.*;

/** Pedro web-tuner wiring on fake hardware; no physical tuning or web UI claim. */
public class PedroTuningHardwareTest {
    private static final String[] WHEELS = {"leftfront", "rightfront", "leftback", "rightback"};
    private final FakeHardwareMap map = new FakeHardwareMap();
    private FakeGoBildaPinpoint pinpoint;

    @Before public void hardware() {
        for (String name : WHEELS) map.register(name, new FakeMotor(2800, 8, 28));
        pinpoint = new FakeGoBildaPinpoint();
        map.register("pinpoint", pinpoint);
    }

    private DcMotorEx wheel(String name) { return map.get(DcMotorEx.class, name); }

    @Test public void tunerDrivesAtFullPowerWhileVerificationKeepsTheAutonomousCap() {
        Drivetrain tuner = PedroTuningHardware.tunerDrivetrain(map);
        tuner.drive(new DrivePowers(1, 0, 0), false);
        for (String name : WHEELS) {
            assertEquals(name, 1, wheel(name).getPower(), 1e-9);
            assertEquals(name, DcMotor.RunMode.RUN_WITHOUT_ENCODER, wheel(name).getMode());
            assertEquals(name, DcMotor.ZeroPowerBehavior.BRAKE, wheel(name).getZeroPowerBehavior());
        }
        Drivetrain capped = PedroTuningHardware.autonomousDrivetrain(map);
        capped.drive(new DrivePowers(1, 0, 0), false);
        for (String name : WHEELS) assertEquals(name, PedroConfiguration.MAX_WHEEL_POWER, wheel(name).getPower(), 1e-9);
        capped.stop();
        for (String name : WHEELS) assertEquals(name, 0, wheel(name).getPower(), 0);
    }

    @Test public void localizerUpdateCostsExactlyOnePinpointOdometryUpdateAndSeedsInches() {
        // The device's first-ever read costs extra; compare the steady per-loop cost after it.
        PinpointOdometry reference = new PinpointOdometry(map);
        reference.update();
        int before = pinpoint.busReadCount();
        reference.update();
        int readsPerOdometryUpdate = pinpoint.busReadCount() - before;
        assertTrue(readsPerOdometryUpdate > 0);

        Localizer localizer = PedroTuningHardware.localizer(map);
        for (int loop = 0; loop < 3; loop++) {
            before = pinpoint.busReadCount();
            localizer.update();
            assertEquals("loop " + loop, readsPerOdometryUpdate, pinpoint.busReadCount() - before);
        }

        before = pinpoint.busReadCount();
        localizer.setPose(new com.pedropathing.math.Pose(12, -6, Math.PI / 2));
        assertEquals(12, localizer.pose().x(), 1e-9);
        assertEquals(-6, localizer.pose().y(), 1e-9);
        assertEquals(Math.PI / 2, localizer.pose().heading(), 1e-9);
        assertEquals(before, pinpoint.busReadCount()); // Seeding is a write, not an extra read.
    }

    /** Mirrors the web tuner's TunerScanner rules so a bad registration fails here, not on the robot. */
    @Test public void everyTunerIsAStaticNoArgumentProcedureFactory() throws Exception {
        List<String> names = new ArrayList<>();
        for (Method method : Tuning.class.getDeclaredMethods()) {
            if (!method.isAnnotationPresent(Tuner.class)) continue;
            assertTrue(method.getName(), Modifier.isStatic(method.getModifiers()));
            assertEquals(method.getName(), 0, method.getParameterCount());
            assertEquals(method.getName(), Procedure.class, method.getReturnType());
            Procedure procedure = (Procedure) method.invoke(null);
            assertNotNull(method.getName(), procedure);
            names.add(procedure.name);
        }
        assertEquals(4, names.size());
        assertTrue(names.containsAll(java.util.Arrays.asList("Mecanum Tuner", "Pinpoint Tuner", "Foresight Tuner", "Tests")));
    }
}
