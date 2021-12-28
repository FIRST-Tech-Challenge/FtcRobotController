package org.firstinspires.ftc.teamcode.main.utils.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * A Test is a piece of code that can be ran to test other code. Tests will be ran automatically by the Tester unless they have an @Skippable annotation.
 * @implSpec Tests should be able to be fully autonomous. A Test's environment should be fully controlled by the Test. Tests should set up their environments in their constructor.
 */
public abstract class Test {

    public abstract boolean isUnitTest();

    public abstract boolean isIntegrationTest();

    /**
     * This is the method to determine the actions of the Test. For example, if you need to send an input to an InputSpace, do so here.
     */
    public abstract void when();

    /**
     * This is the method to determine the outcome of the Test. For example, if you need to receive output from an OutputSpace and check if it's true, do so here.
     */
    public abstract boolean then();

    public abstract String getLogs();

    public String getName() {
        return getClass().getName();
    }

    /**
     * Since Tests are run inside an OpMode, they can access it. Be careful as to not stop or break the OpMode though.
     * @return {@link} TestOpMode
     */
    public LinearOpMode getOpMode() {
        return TestResources.opMode;
    }

    public String toString() {
        return "Test: " + getName() + System.lineSeparator() + "-----------------------------------------------" + System.lineSeparator() + "Logs:" + System.lineSeparator() + getLogs();
    }

}
