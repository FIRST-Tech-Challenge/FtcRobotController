package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.nknsd.robotics.framework.NKNProgram;
import org.nknsd.robotics.team.SampleNKNProgram;

import java.util.concurrent.TimeUnit;

@TeleOp(name = "Sample NKNOpMode")
public class NKNOpMode_Iterative extends OpMode {
    private final ElapsedTime runtime = new ElapsedTime();

    //Time counters are in milliseconds
    private long lastTelemetryCall = 0;
    private final long TELEMETRY_DELAY = 500;

    // load your program here
    private final NKNProgram program = new SampleNKNProgram();


    // Code to run ONCE when the driver hits INIT
    @Override
    public void init() {
        runtime.reset();
        telemetry.addData("Status", "Initializing");
        telemetry.update();

        program.init(telemetry, hardwareMap, gamepad1, gamepad2);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    // Code to run REPEATEDLY after hitting INIT but before hitting PLAY
    @Override
    public void init_loop() {program.init_loop(runtime, telemetry);}

    // Code to run ONCE when the driver hits PLAY
    @Override
    public void start() {
        runtime.reset();
        program.start(runtime, telemetry);
    }

    // Code to run REPEATEDLY after the driver hits PLAY
    //Does NOT handle telemetry
    @Override
    public void loop() {
        program.loop(runtime, telemetry);

        if (runtime.now(TimeUnit.MILLISECONDS) - TELEMETRY_DELAY > lastTelemetryCall) {
            doTelemetry();
            lastTelemetryCall = runtime.now(TimeUnit.MILLISECONDS);
        }
    }

    public void doTelemetry() {
        program.doTelemetry(telemetry);
    }

    // Code to run ONCE after the driver hits STOP
    @Override
    public void stop() {program.stop(runtime, telemetry);}

}
