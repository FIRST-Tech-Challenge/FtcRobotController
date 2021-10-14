package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.event.Event;
import org.firstinspires.ftc.teamcode.core.event.EventThread;
import org.firstinspires.ftc.teamcode.core.movement.api.StrafingMovement;
import org.firstinspires.ftc.teamcode.core.movement.impl.StrafedMovementImpl;

import java.util.concurrent.CompletableFuture;
import java.util.concurrent.atomic.AtomicBoolean;

@Autonomous
public class EventTest extends LinearOpMode {
    public EventThread eventThread = new EventThread();

    @Override
    public void runOpMode() throws InterruptedException {
        int runTime = 1000;
        double power = 0.3;
        int restTime = 1000;

        waitForStart();

        Object object = new Object();
        StrafingMovement strafedMovement = new StrafedMovementImpl(hardwareMap);
        telemetry.addLine("Driving forward for " + runTime / 1000 + " second(s).");
        telemetry.update();
        strafedMovement.drive(power);

        eventThread.addEvent(Event.createEventWithMilis(() -> {
            strafedMovement.stop();
            object.notifyAll();
        }, runTime));

        object.wait();


    }
}
