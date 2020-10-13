package org.firstinspires.ftc.teamcode.opMode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManager;

import org.firstinspires.ftc.robotcontroller.internal.FtcOpModeRegister;
import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * A driver controlled program.
 * NOTE: This class will automatically stop itself two minutes after starting!
 * Usage:
 * This class should be extended by a single program in which you MUST:
 *  - Override #buildRobot()
 * and MAY:
 *  - Override #onStart()
 *  - Override #onUpdate()
 *  - Override #onStop()
 * @Author Jaxon Brown
 */
public abstract class DriverControlledProgram extends OpMode {
    private Robot robot;
    private OpModeManager opModeManager;
    private long stoppingTime;
    private boolean timerDisabled = false;

    /**
     * Build a robot. This should be overridden by your Program.
     * Construct your robot and make any necessary changes to the subsystems.
     * @return Robot this program controls.
     */
    protected abstract Robot buildRobot();

    /**
     * Called when the the program is started.
     */
    protected void onStart() {}

    /**
     * Called when the loop finishes.
     */
    protected void onUpdate() {}

    /**
     * Called when the robot is stopped.
     */
    protected void onStop() {}

    @Override
    public final void init() {


        robot = buildRobot();

        try {
            robot.init();
        } catch(Exception ex) {
            telemetry.addData("ERROR!!!", ex.getMessage());
        }
    }

    @Override
    public final void start() {
        stoppingTime = System.currentTimeMillis() + 1000 * 121;

        onStart();
    }

    @Override
    public final void loop() {


        telemetry.addData("Time", (stoppingTime - System.currentTimeMillis())/1000D);

        robot.driverControlledUpdate();
        onUpdate();
    }

    @Override
    public final void stop() {
        onStop();
    }

    /**
     * Gets the robot. If you properly cached your subcomponents in buildRobot(), you probably don't need this.
     * @return
     */
    protected final Robot getRobot() {
        return robot;
    }

    protected void disableTimer() {
        this.timerDisabled = true;
    }
}