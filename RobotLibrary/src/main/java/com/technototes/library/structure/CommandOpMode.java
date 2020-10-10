package com.technototes.library.structure;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.technototes.library.command.Command;
import com.technototes.library.command.CommandScheduler;
import com.technototes.library.command.InstantCommand;
import com.technototes.library.control.gamepad.CommandGamepad;
import com.technototes.library.hardware.HardwareDevice;
import com.technototes.logger.Logger;

public abstract class CommandOpMode extends LinearOpMode {
    public static double commandTimeAtEnd = 5;
    public CommandGamepad driverGamepad;
    public CommandGamepad codriverGamepad;
    public ElapsedTime timer = new ElapsedTime();
    public OpModeState opModeState = OpModeState.INIT;
    public Logger logger;

    public OpModeState getOpModeState() {
        return opModeState;
    }

    @Override
    public final void runOpMode() throws InterruptedException {
        driverGamepad = new CommandGamepad(gamepad1);
        codriverGamepad = new CommandGamepad(gamepad2);
        HardwareDevice.hardwareMap = hardwareMap;
        beginInit();
        logger = new Logger(telemetry, this);
        while (!isStarted()) {
            beginLoop();
            CommandScheduler.getInitInstance().run();
            logger.update();
            telemetry.update();
        }
        CommandScheduler.getInitInstance().runLastTime();
        opModeState = OpModeState.RUN;
        runInit();
        while (opModeIsActive()) {
            runLoop();
            CommandScheduler.getRunInstance().run();
            logger.update();
            telemetry.update();
            driverGamepad.periodic();
            codriverGamepad.periodic();
        }
        CommandScheduler.getRunInstance().runLastTime();
        opModeState = OpModeState.FINISHED;
        end();
        timer.reset();
        CommandScheduler.getEndInstance().run();
        CommandScheduler.getEndInstance().runLastTime();
    }

    //for registering commands to run when robot is in init
    public void beginInit() {

    }

    //for other things to do in init
    public void beginLoop() {

    }

    //to schedule commands to be run
    public void runInit() {

    }

    public void runLoop() {

    }

    public void end() {

    }

    public enum OpModeState {
        INIT, RUN, FINISHED
    }

    public CommandScheduler schedule(Command c){
        switch (opModeState){
            case INIT:
                return CommandScheduler.getInitInstance().schedule(c);
            case FINISHED:
                return CommandScheduler.getEndInstance().schedule(c);
            default:
                return CommandScheduler.getRunInstance().schedule(c);
        }
    }
    public CommandScheduler schedule(Runnable c){
        return schedule(new InstantCommand(c));
    }

}
