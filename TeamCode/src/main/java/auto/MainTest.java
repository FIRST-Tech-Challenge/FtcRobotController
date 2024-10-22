// MainTest.java
package auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "MainTest", group = "Test")
public class MainTest extends LinearOpMode {
    private Robot robot;
    private DcMotor slideH;

    @Override
    public void runOpMode() {
        robot = new Robot();
        slideH = hardwareMap.get(DcMotor.class, "slideH");
        HSlide sliderH = new HSlide(slideH);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {
            // Add autonomous actions here
            robot.resetAll();
        }
    }

    // Add the resetPosition method e
    public void resetPosition() {
        // Base implementation or abstract method
    }
}

