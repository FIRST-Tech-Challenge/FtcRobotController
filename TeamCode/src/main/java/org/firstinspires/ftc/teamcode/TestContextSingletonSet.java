
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/*
This is a copy of the FirstAutonomousIterationMirror just the last portion to make sure we go park

 */


@Autonomous(name="Test: Context Set", group="Test")
//@Disabled

public class TestContextSingletonSet extends LinearOpMode {

    public TestContextSingletonSet() {
    }

    @Override
    public void runOpMode() throws InterruptedException {
        ContextSingleton.getContext().setHeading(100.0);
    }
}
