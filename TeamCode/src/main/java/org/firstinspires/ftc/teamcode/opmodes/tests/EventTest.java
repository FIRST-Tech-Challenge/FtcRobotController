package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.thread.old.types.impl.TimedEvent;
import org.firstinspires.ftc.teamcode.core.thread.old.EventThread;

import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous
@Disabled
public class EventTest extends LinearOpMode {
    public EventThread eventThread = new EventThread();

    public EventTest() {
        this.msStuckDetectStart = Integer.MAX_VALUE;
        this.msStuckDetectInit = Integer.MAX_VALUE;
        this.msStuckDetectInitLoop = Integer.MAX_VALUE;
        this.msStuckDetectLoop = Integer.MAX_VALUE;
        this.msStuckDetectStop = Integer.MAX_VALUE;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        int runTime = 5;
//        double power = 0.3;
//        int restTime = 1000;

        eventThread.start();
        waitForStart();

//        StrafingMovement strafedMovement = new StrafedMovementImpl(hardwareMap);
        telemetry.addLine("Doing nothing for " + runTime + " second(s).");
        telemetry.update();
//        strafedMovement.drive(power);


        AtomicBoolean atomicBoolean = new AtomicBoolean();

//            strafedMovement.stop();
        eventThread.addEvent(TimedEvent.createEventWithSeconds(() -> atomicBoolean.set(true), runTime));
        while (opModeIsActive()) {
            if (atomicBoolean.get()) break;
        }
        telemetry.addLine("done!");
        telemetry.update();
        requestOpModeStop();
    }
}
