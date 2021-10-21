package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.event.Event;
import org.firstinspires.ftc.teamcode.core.event.EventThread;
import org.firstinspires.ftc.teamcode.core.movement.api.StrafingMovement;
import org.firstinspires.ftc.teamcode.core.movement.impl.StrafedMovementImpl;

import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous
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
        int runTime = 1000;
//        double power = 0.3;
//        int restTime = 1000;

        eventThread.start();
        waitForStart();

//        StrafingMovement strafedMovement = new StrafedMovementImpl(hardwareMap);
        telemetry.addLine("Doing nothing for " + runTime / 1000 + " second(s).");
        telemetry.update();
//        strafedMovement.drive(power);

        AtomicBoolean atomicBoolean = new AtomicBoolean();

        //            strafedMovement.stop();
        eventThread.addEvent(Event.createEventWithMilis(() -> {
            telemetry.addLine("done!");
            telemetry.update();
            atomicBoolean.set(true);
        }, runTime));

        while (true) {
            if(atomicBoolean.get()) break;
        }
    }
}
