package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.doAnswer;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.LinkedHashMap;
import java.util.Map;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.bot.BotProperties;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/**
 * Aim assist through the real drive code: a pretend camera says where the tag is, and the
 * four pretend motors show which way the robot would turn. Full speed, heading 0, so the
 * numbers are easy to read: a left turn is left wheels forward, right wheels backward.
 */
@DisplayName("Drive_Manual aim assist")
class DriveAimTest {
    private static final double EPS = 1e-9;
    private final BotProperties properties = new BotProperties();
    private final Map<String, double[]> power = new LinkedHashMap<>();
    private HydraOpMode op;
    private Gamepad gamepad;
    private Vision vision;
    private Drive_Manual drive;

    private DcMotorEx motor(String name) {
        double[] last = {0};
        power.put(name, last);
        DcMotorEx m = mock(DcMotorEx.class);
        doAnswer(inv -> {
            last[0] = inv.getArgument(0);
            return null;
        }).when(m).setPower(anyDouble());
        when(m.getPower()).thenAnswer(inv -> last[0]);
        return m;
    }

    @BeforeEach
    void setUp() {
        HardwareMap hardwareMap = mock(HardwareMap.class);
        for (String name : new String[] {
                properties.cfgDriveFrontLeft, properties.cfgDriveFrontRight,
                properties.cfgDriveBackLeft, properties.cfgDriveBackRight}) {
            DcMotorEx m = motor(name);   // build the mock first: no stubbing inside a stubbing
            when(hardwareMap.get(DcMotorEx.class, name)).thenReturn(m);
        }
        gamepad = mock(Gamepad.class);
        Imu imu = mock(Imu.class);
        when(imu.Ready()).thenReturn(true);
        when(imu.GetYaw()).thenReturn(0.0);
        vision = mock(Vision.class);
        op = new HydraOpMode(mock(Telemetry.class), hardwareMap, gamepad, mock(Gamepad.class));
        op.mSafeMode = false;
        drive = new Drive_Manual(op, properties, imu, vision);
    }

    /** Enough loops for the slew rate to reach any target. */
    private void runLoops() {
        for (int i = 0; i < 50; i++) {
            drive.HandleUserInput();
            drive.Process();
        }
    }

    private double frontLeft() {
        return power.get(properties.cfgDriveFrontLeft)[0];
    }

    private double frontRight() {
        return power.get(properties.cfgDriveFrontRight)[0];
    }

    private double backLeft() {
        return power.get(properties.cfgDriveBackLeft)[0];
    }

    private double backRight() {
        return power.get(properties.cfgDriveBackRight)[0];
    }

    private void tagAt(double bearing) {
        when(vision.SeesTag()).thenReturn(true);
        when(vision.GetBearing()).thenReturn(bearing);
        when(vision.GetTagId()).thenReturn(21);
    }

    @Test
    @DisplayName("left bumper held, tag 20 degrees to the left: the robot turns left by itself")
    void turnsLeftTowardTag() {
        gamepad.left_bumper = true;
        tagAt(20);
        runLoops();
        // kP 0.02 x 20 degrees = 0.4, with nothing on the sticks.
        assertEquals(0.4, frontLeft(), EPS);
        assertEquals(0.4, backLeft(), EPS);
        assertEquals(-0.4, frontRight(), EPS);
        assertEquals(-0.4, backRight(), EPS);
    }

    @Test
    @DisplayName("tag to the right: turns right")
    void turnsRightTowardTag() {
        gamepad.left_bumper = true;
        tagAt(-20);
        runLoops();
        assertEquals(-0.4, frontLeft(), EPS);
        assertEquals(0.4, frontRight(), EPS);
    }

    @Test
    @DisplayName("a right turn from aim assist is the same direction as pushing the stick right")
    void sameDirectionAsTheStick() {
        gamepad.right_stick_x = 1;
        runLoops();
        double stickFrontLeft = frontLeft();
        assertTrue(stickFrontLeft < 0, "stick right should give front left " + stickFrontLeft);

        gamepad.right_stick_x = 0;
        gamepad.left_bumper = true;
        tagAt(-20);
        runLoops();
        assertTrue(frontLeft() < 0, "tag on the right should turn the same way as stick right");
    }

    @Test
    @DisplayName("bumper held but no tag in view: the stick still turns the robot")
    void noTagStickStillWorks() {
        gamepad.left_bumper = true;
        when(vision.SeesTag()).thenReturn(false);
        gamepad.right_stick_x = 1;
        runLoops();
        assertTrue(frontLeft() < 0);
        assertTrue(frontRight() > 0);
    }

    @Test
    @DisplayName("bumper not held: the tag is ignored and the robot stays still")
    void notHeldIgnoresTag() {
        gamepad.left_bumper = false;
        tagAt(20);
        runLoops();
        assertEquals(0, frontLeft(), EPS);
        assertEquals(0, frontRight(), EPS);
    }

    @Test
    @DisplayName("tag almost centered: no turning, so the robot does not wobble")
    void centeredTagNoTurn() {
        gamepad.left_bumper = true;
        tagAt(0.5);
        runLoops();
        assertEquals(0, frontLeft(), EPS);
        assertEquals(0, frontRight(), EPS);
    }
}
