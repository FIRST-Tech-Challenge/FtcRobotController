package org.firstinspires.ftc.teamcode;

import org.junit.Test;
import static org.junit.Assert.*;

public class HiveTagGeometryTest {
    @Test public void nominalMapFusesIndividualTagPositionsAtUpdatedHeight() {
        ShooterMount mount = ControllerFixtures.mount();
        for (TurretSubsystem.Alliance alliance : TurretSubsystem.Alliance.values()) {
            for (TurretSubsystem.HiveSide side : TurretSubsystem.HiveSide.values()) {
                GoalTarget target = TeleOpFieldReference.target(alliance, side);
                for (GoalTarget.Tag known : target.tags.values()) {
                    PoseHistory history = new PoseHistory();
                    double actualX = known.x - 2, actualY = known.y;
                    // Settled history: fusion rejects frames captured while the turret/chassis turns.
                    for (double t = .88; t < 1.001; t += .02)
                        history.add(ControllerFixtures.pose(Math.min(t,1), actualX + .1, actualY, 0, 0, 0, 0), 0);
                    TagObservation observed = TagFixtures.seen(target, known.id,
                            ControllerFixtures.pose(1, actualX, actualY, 0, 0, 0, 0), 0, mount);
                    assertTrue(history.reason(), history.observe3d(1, 1, 1, observed, target, mount));
                    assertEquals(actualX + .065, history.current(1).x, 1e-9);
                    assertEquals(actualY, history.current(1).y, 1e-9);
                }
            }
        }
    }
    @Test public void dimensionsMatchManualAndSdkUsesDifferentOrigin() {
        assertEquals(.08255, HiveTagGeometry.TAG_SIZE_METERS, 1e-9);
        assertEquals(.1825752, HiveTagGeometry.FRONT_TO_TAG_ROW_METERS, 1e-9);
        assertEquals(.3138318447, HiveTagGeometry.RAISED_TAG_ROW_Y_METERS, 1e-9);
        assertEquals(1.2676124, HiveTagGeometry.DRAWING_TAG_HEIGHT_METERS, 1e-9);
        assertEquals(.18255996, HiveTagGeometry.SDK_MEMBER_Y_METERS, 1e-9);
        assertEquals(-.1427988, HiveTagGeometry.SDK_MEMBER_Z_METERS, 1e-9);
        assertThrows(IllegalArgumentException.class, () -> HiveTagGeometry.offsetFromStickerCenterMeters(29));
    }
    @Test public void allSixteenTagsHaveCorrectAllianceRowAndLeftRightOrdering() {
        double[][] expectedX = {
                // Upright stickers (inverted-mount conversion, physical frame 2026-11-13): IDs increase
                // left to right, so far IDs decrease with X and audience IDs increase with X.
                {-.15875,-.254,-.3937,-.48895}, {-.48895,-.3937,-.254,-.15875},
                {.15875,.254,.3937,.48895}, {.48895,.3937,.254,.15875}};
        for (int row = 0; row < 4; row++) {
            TurretSubsystem.Alliance alliance = row < 2 ? TurretSubsystem.Alliance.RED : TurretSubsystem.Alliance.BLUE;
            TurretSubsystem.HiveSide side = row == 0 || row == 3 ? TurretSubsystem.HiveSide.FAR : TurretSubsystem.HiveSide.AUDIENCE;
            GoalTarget.Tag[] tags = HiveTagGeometry.nominalRaisedTags(alliance, side);
            for (int i = 0; i < 4; i++) {
                assertEquals(30 + row * 4 + i, tags[i].id);
                assertEquals(expectedX[row][i], tags[i].x, 1e-9);
                assertEquals(side == TurretSubsystem.HiveSide.FAR ? 1 : -1, Math.signum(tags[i].y), 0);
                assertEquals(1.2676124, tags[i].height, 1e-9); // Drawing-derived nominal raised tag height.
                assertEquals(side, TurretSubsystem.getHiveSideForTag(alliance, tags[i].id));
            }
        }
    }
}
