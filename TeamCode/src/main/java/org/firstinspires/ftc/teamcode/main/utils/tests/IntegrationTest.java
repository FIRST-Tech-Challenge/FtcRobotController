package org.firstinspires.ftc.teamcode.main.utils.tests;

/**
 * An IntegrationTest is a Test comprised of multiple UnitTests to assure multiple features can be used.
 */
public abstract class IntegrationTest extends Test {

    @Override
    public boolean isUnitTest() {
        return false;
    }

    @Override
    public boolean isIntegrationTest() {
        return true;
    }

}
