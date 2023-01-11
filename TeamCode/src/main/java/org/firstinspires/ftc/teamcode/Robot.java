package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.dragonswpilib.command.CommandScheduler;

@TeleOp
public class Robot extends OpMode{

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        CommandScheduler.getInstance().reset();
        new RobotContainer(gamepad1, gamepad2, telemetry, hardwareMap);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
        CommandScheduler.getInstance().run();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        CommandScheduler.getInstance().run();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        CommandScheduler.getInstance().reset();
    }
}
