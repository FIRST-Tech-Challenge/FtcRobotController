package org.firstinspires.ftc.teamcode.opmodes.mechBot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Robot2;
import org.firstinspires.ftc.teamcode.hardware.MechBot.ToboMech;
import org.firstinspires.ftc.teamcode.hardware.Sigma.ToboSigma;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.events.EventManager;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;
import org.firstinspires.ftc.teamcode.support.tasks.TaskManager;

import java.io.IOException;

/**
 * Created by 28761 on 6/29/2019.
 */
//@Disabled
@Autonomous(name = "Auto-Menu", group = "MechBot")
public class AutoMenu extends LinearOpMode {
    private ToboSigma.SkystoneLocation StoneLoc;
    private EventManager em1;
    protected static int LOG_LEVEL = Log.INFO;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    public double auto_chassis_power = .4;

    @Override
    public void runOpMode() throws InterruptedException {
        log.info("RoboMech Autonomous runOpMode() starts (CPU_time = %.2f sec)", getRuntime());
        telemetry.addData("Initializing Robot", "Please Wait ...");
        telemetry.update();

        ToboMech robot = new ToboMech();
        em1 = new EventManager(gamepad1, true);
        robot.useTfod = true;
        // robot.set_simulation_mode(true);
        robot.configureLogging("ToboMech", LOG_LEVEL);
        configuration = new Configuration(hardwareMap, robot.getName()).configureLogging("Config", LOG_LEVEL);
        log.info("RoboMech Autonomous finished log configuration (CPU_time = %.2f sec)", getRuntime());

        try {
            // configure robot and reset all hardware

            robot.configure(configuration, telemetry, Robot2.ProgramType.AUTO_BLUE);
            configuration.apply();
            robot.initSetup(Robot2.ProgramType.AUTO_BLUE, ToboMech.StartPosition.OUT, configuration); // check
            robot.reset(true);
            robot.showInitStatus(getRuntime());
        } catch (Exception E) {
            telemetry.addData("Init Failed", E.getMessage());
            handleException(E);
        }
        log.info("RoboMech Autonomous finished initialization (CPU_time = %.2f sec)", getRuntime());

        robot.AutoMenu(em1);
        while (!robot.autoPara.isDone()) { // push X will exit the loop
            try {
                em1.processEvents();
                TaskManager.processTasks();
                if (Thread.interrupted()) {
                    return;
                }
            } catch (Exception E) {
                telemetry.addData("Error in event handler", E.getMessage());
                handleException(E);
                Thread.sleep(5000);
            }
        }

        robot.showStatus(getRuntime());

        waitForStart();

        if (!robot.isSimulationMode())
            if (robot.shooter!= null) {
                robot.shooter.shootOutByRpm(ToboMech.WARM_UP_RPM_AUTO);
            }
            robot.initAfterStart();

        robot.runtime.reset();
        robot.runtimeAuto.reset();
        // run until the end of the match (driver presses STOP or timeout)
        if (opModeIsActive()) {
            try {
                robot.detectPosition();
                if (robot.autoPara.isDoPowerShots() && robot.tZone!= ToboMech.TargetZone.ZONE_C) {
                    robot.deliverFirstWobbleGoal();
                    if ((robot.runtimeAuto.seconds() < 20) && opModeIsActive()) {
                        robot.doPowerShots();
                        if ((robot.runtimeAuto.seconds() < 25) && opModeIsActive()){
                            robot.getSecondWobbleGoal();
                            robot.deliverSecondWobbleGoalAndShootBonusRings();
                        }
                    }
                    robot.park();
                } else {
                    robot.shooter.shootOutByRpm(ToboMech.WARM_UP_RPM_AUTO);
                    // write the program here
                    //if ((robot.runtimeAuto.seconds() < 29.5) && opModeIsActive()
                    robot.detectPosition();
                    robot.doHighGoalsAndPowerShots(3, 0, false);
                    robot.deliverFirstWobbleGoalAfterHighGoal();
                    if ((robot.runtimeAuto.seconds() < 25) && opModeIsActive()){
                        robot.getSecondWobbleGoalAfterHighGoal();
                        robot.deliverSecondWobbleGoalAndShootBonusRings();
                    }

                    robot.park();
                }
            } catch (Exception E) {
                telemetry.addData("Error in event handler", E.getMessage());
                handleException(E);
                Thread.sleep(5000);
            }
        }
        try {
            robot.end();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    protected void handleException(Throwable T) {
        log.error(T.getMessage(), T);
        int linesToShow = 5;
        for (StackTraceElement line : T.getStackTrace()) {
            telemetry.log().add("%s.%s():%d", line.getClassName(), line.getMethodName(), line.getLineNumber());
            if (--linesToShow == 0) break;
        }
        telemetry.update();
    }
}
