package org.firstinspires.ftc.teamcode.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;
import static org.mockito.ArgumentMatchers.anyInt;
import static org.mockito.ArgumentMatchers.contains;
import static org.mockito.ArgumentMatchers.endsWith;
import static org.mockito.Mockito.never;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.times;
import static org.mockito.Mockito.verify;
import static org.mockito.Mockito.when;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.util.Arrays;
import java.util.Collections;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.teamcode.bot.BotProperties;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.DisplayName;
import org.junit.jupiter.api.Test;

/** The Limelight camera with pretend results. */
@DisplayName("Vision_Limelight")
class VisionLimelightTest {
    private static final double EPS = 1e-9;
    private final BotProperties properties = new BotProperties();
    private Limelight3A limelight;
    private Telemetry telemetry;
    private Vision_Limelight vision;

    @BeforeEach
    void setUp() {
        HardwareMap hardwareMap = mock(HardwareMap.class);
        limelight = mock(Limelight3A.class);
        when(hardwareMap.get(Limelight3A.class, properties.cfgLimelight)).thenReturn(limelight);
        telemetry = mock(Telemetry.class);
        HydraOpMode op = new HydraOpMode(telemetry, hardwareMap, mock(Gamepad.class), mock(Gamepad.class));
        vision = new Vision_Limelight(op, properties);
    }

    /** A tag the Limelight reports: its id, degrees right of center, and how much of the picture it fills. */
    private LLResultTypes.FiducialResult tag(int id, double txDegreesRight, double areaPercent) {
        LLResultTypes.FiducialResult tag = mock(LLResultTypes.FiducialResult.class);
        when(tag.getFiducialId()).thenReturn(id);
        when(tag.getTargetXDegrees()).thenReturn(txDegreesRight);
        when(tag.getTargetArea()).thenReturn(areaPercent);
        return tag;
    }

    private void limelightSees(LLResultTypes.FiducialResult... tags) {
        LLResult result = mock(LLResult.class);
        when(result.isValid()).thenReturn(true);
        when(result.getFiducialResults()).thenReturn(Arrays.asList(tags));
        when(limelight.getLatestResult()).thenReturn(result);
    }

    @Test
    @DisplayName("Init starts the Limelight once, on the AprilTag pipeline")
    void initStartsOnce() {
        assertTrue(vision.Init());
        assertTrue(vision.Init());
        verify(limelight, times(1)).start();
        verify(limelight, times(1)).pipelineSwitch(properties.limelightAprilTagPipeline);
        verify(limelight, times(1)).setPollRateHz(anyInt());
    }

    @Test
    @DisplayName("no result yet: sees nothing")
    void noResult() {
        when(limelight.getLatestResult()).thenReturn(null);
        vision.Process();
        assertFalse(vision.SeesTag());
        assertEquals(0, vision.GetTagId());
        assertEquals(0, vision.CountTags());
    }

    @Test
    @DisplayName("a result with no tags in it: sees nothing")
    void noTags() {
        LLResult result = mock(LLResult.class);
        when(result.isValid()).thenReturn(true);
        when(result.getFiducialResults()).thenReturn(Collections.emptyList());
        when(limelight.getLatestResult()).thenReturn(result);
        vision.Process();
        assertFalse(vision.SeesTag());
    }

    @Test
    @DisplayName("picks the tag that fills the most of the picture: the nearest one")
    void picksTheBiggest() {
        limelightSees(tag(21, 10, 2.0), tag(24, -5, 5.0), tag(22, 0, 1.0));
        vision.Process();
        assertTrue(vision.SeesTag());
        assertEquals(24, vision.GetTagId());
        assertEquals(3, vision.CountTags());
    }

    @Test
    @DisplayName("lists every tag in view on the Driver Station, and marks the nearest one")
    void listsEveryTag() {
        limelightSees(tag(21, 10, 2.0), tag(24, -5, 5.0));
        vision.Process();
        verify(telemetry).addLine(contains("tag 21: fills 2.0%"));
        verify(telemetry).addLine(contains("tag 24: fills 5.0%"));
        verify(telemetry).addLine(endsWith("<- nearest"));
        verify(telemetry, never()).addLine(contains("tag 21: fills 2.0%, 10.0 deg to the right  <- nearest"));
    }

    @Test
    @DisplayName("bearing counts left as positive: the Limelight's right-positive tx is flipped")
    void bearingLeftPositive() {
        limelightSees(tag(21, 10, 1.0));
        vision.Process();
        assertEquals(-10, vision.GetBearing(), EPS);
    }

    @Test
    @DisplayName("range comes from the Limelight's 3D position, in meters, turned into inches")
    void rangeFromPose() {
        Pose3D pose = new Pose3D(new Position(DistanceUnit.METER, 0.3, 0.4, 0, 0), null);
        assertEquals(DistanceUnit.METER.toInches(0.5), Vision_Limelight.RangeInches(pose), 1e-6);
    }

    @Test
    @DisplayName("no 3D position: range is 0, not a crash")
    void noPoseNoCrash() {
        assertEquals(0, Vision_Limelight.RangeInches(null), EPS);
        assertEquals(0, Vision_Limelight.RangeInches(new Pose3D(null, null)), EPS);
    }
}
