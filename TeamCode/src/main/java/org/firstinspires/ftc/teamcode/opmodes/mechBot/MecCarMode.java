package org.firstinspires.ftc.teamcode.opmodes.mechBot;

import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.components.Robot2;
import org.firstinspires.ftc.teamcode.hardware.MechBot.ToboMech;
import org.firstinspires.ftc.teamcode.support.Logger;
import org.firstinspires.ftc.teamcode.support.events.EventManager;
import org.firstinspires.ftc.teamcode.support.hardware.Configuration;

@Disabled
@TeleOp(name="Mec-CarOp", group="MechBot")
public class MecCarMode extends LinearOpMode {
    protected static int LOG_LEVEL = Log.INFO;

    private Configuration configuration;
    private Logger<Logger> log = new Logger<Logger>().configureLogging(getClass().getSimpleName(), LOG_LEVEL);

    private EventManager eventManager1;
    private EventManager eventManager2;

    @Override
    public void runOpMode() throws InterruptedException {
        log.info("RoboRuck TeleOp runOpMode() starts (CPU_time = %.2f sec)", getRuntime());
        telemetry.addData("Initializing Robot", "Please Wait ...");
        telemetry.update();

        ToboMech robot = new ToboMech();
        robot.configureLogging(robot.getName(),LOG_LEVEL);
        configuration = new Configuration(hardwareMap, robot.getName()).configureLogging("Config", LOG_LEVEL);
        log.info("RoboRuck TeleOp finished configuration (CPU_time = %.2f sec)", getRuntime());

        try {
            // configure robot and reset all hardware
            robot.configure(configuration, telemetry, Robot2.ProgramType.TELE_OP);
            configuration.apply();
            robot.reset(false);

            eventManager1 = new EventManager(gamepad1, true);
            eventManager2 = new EventManager(gamepad2, true);

            robot.mainTeleOp(eventManager1);

            telemetry.addData("Robot is ready", "Press Play");
            telemetry.update();
        } catch (Exception E) {
            telemetry.addData("Init Failed", E.getMessage());
            handleException(E);
        }
        log.info("RoboRuck TeleOp finished initialization (CPU_time = %.2f sec)", getRuntime());
        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            robot.chassis.carDrive(gamepad1.left_stick_y,gamepad1.right_stick_x);

        }
    }

    protected void handleException(Throwable T) {
        log.error(T.getMessage(), T);
        int linesToShow = 5;
        for(StackTraceElement line : T.getStackTrace()) {
            telemetry.log().add("%s.%s():%d", line.getClassName(), line.getMethodName(), line.getLineNumber());
            if (--linesToShow == 0) break;
        }
        telemetry.update();
    }
}
