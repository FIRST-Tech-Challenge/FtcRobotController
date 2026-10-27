package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.RobotHardware;

/**
 * Autonomous mode code goes here
 */
@Autonomous(name="Autonomous Mode", group="Robot")
public class AutoMode extends RobotOpMode {

    @Override
    public void init() {
        telemetryPrinter.helloWorld("Chameleons");
    }

    @Override
    public void init_loop() {
        telemetryPrinter.helloWorld("Chameleons");
    }

    @Override
    public void start() {
        telemetryPrinter.print("Staring AutoMode...");

    }

    @Override
    public void loop() {
        telemetryPrinter.helloWorld("Chameleons on the bot");
    }

    @Override
    public void stop() {
        telemetryPrinter.print("Stopping AutoMode...");
    }

}
