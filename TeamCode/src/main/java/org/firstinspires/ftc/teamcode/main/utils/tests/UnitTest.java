package org.firstinspires.ftc.teamcode.main.utils.tests;

/**
 * A UnitTest is a test of a specific piece of the codebase.
 */
public abstract class UnitTest extends Test {

    @Override
    public boolean isUnitTest() {
        return true;
    }

    @Override
    public boolean isIntegrationTest() {
        return false;
    }

}
