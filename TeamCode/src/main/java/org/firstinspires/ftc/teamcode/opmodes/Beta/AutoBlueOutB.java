package org.firstinspires.ftc.teamcode.opmodes.Beta;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.components.Robot2;
import org.firstinspires.ftc.teamcode.hardware.Beta.ToboBeta;
//import org.firstinspires.ftc.teamcode.hardware.MechBot.ToboMech;
import org.firstinspires.ftc.teamcode.hardware.Sigma.ToboSigma;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

import java.io.IOException;

/**
 * Created by 28761 on 6/29/2019.
 */
@Disabled
@Autonomous(name = "Blue Out Beta", group = "Beta")
public class AutoBlueOutB extends LinearOpMode {

    protected static int LOG_LEVEL = Log.INFO;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    @Override
    public void runOpMode() throws InterruptedException {
        log.info("RoboBeta Autonomous runOpMode() starts (CPU_time = %.2f sec)", getRuntime());
        telemetry.addData("Initializing Robot", "Please Wait ...");
        telemetry.update();

        ToboBeta robot = new ToboBeta();
        robot.useTfod = true;
        // robot.set_simulation_mode(true);
        robot.configureLogging("ToboBeta", LOG_LEVEL);
        configuration = new Configuration(hardwareMap, robot.getName()).configureLogging("Config", LOG_LEVEL);
        log.info("RoboBeta Autonomous finished log configuration (CPU_time = %.2f sec)", getRuntime());

        try {
            // configure robot and reset all hardware

            robot.configure(configuration, telemetry, Robot2.ProgramType.AUTO_BLUE);
            configuration.apply();
            robot.initSetup(Robot2.ProgramType.AUTO_BLUE, ToboBeta.StartPosition.OUT, configuration); // check
            robot.reset(true);
            robot.showStatus(getRuntime());
        } catch (Exception E) {
            telemetry.addData("Init Failed", E.getMessage());
            handleException(E);
        }
        log.info("RoboMech Autonomous finished initialization (CPU_time = %.2f sec)", getRuntime());

        waitForStart();

        if (!robot.isSimulationMode())
            if (robot.shooter!= null) {
                robot.shooter.shootOutByRpm(ToboBeta.WARM_UP_RPM_AUTO);
            }
            robot.initAfterStart();

        robot.runtime.reset();
        robot.runtimeAuto.reset();
        // run until the end of the match (driver presses STOP or timeout)
        if (opModeIsActive()) {
            try {
            if (robot.shooter!= null) {
                robot.shooter.shootOutByRpm(ToboBeta.WARM_UP_RPM_AUTO);
            }
                // write the program here
                //if ((robot.runtimeAuto.seconds() < 29.5) && opModeIsActive()
                robot.detectPosition();
                robot.doHighGoalsAndPowerShots(3, 0, false);
                robot.getBonusRingsBeta();
                robot.deliverFirstWobbleGoalBeta();
                if ((robot.runtimeAuto.seconds() < 25) && opModeIsActive()){
                    robot.getSecondWobbleGoalBeta();
                    robot.deliverSecondWobbleGoalBeta();
                }

               robot.park();

            } catch (Exception E) {
                telemetry.addData("Error in event handler", E.getMessage());
                handleException(E);
                Thread.sleep(10000);
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
