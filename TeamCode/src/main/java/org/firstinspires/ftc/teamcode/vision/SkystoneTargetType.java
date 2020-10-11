package org.firstinspires.ftc.teamcode.vision;

/*
Quarry is looking at a quarry wall for initial detection of the skystone positions within the quarry
STONE is looking for an individual stone
FOUNDATION is looking for a column of nubs on the foundation
TOWER is looking for a tower on the foundation
FOUNDATIONSTONE is looking for a single stone on the foundation - assuming that's a bit different that recognization a tower at least 2 stones tall
 */
public enum SkystoneTargetType {
    QUARRY, STONE, FOUNDATION, TOWER, FOUNDATIONSTONE;
}
