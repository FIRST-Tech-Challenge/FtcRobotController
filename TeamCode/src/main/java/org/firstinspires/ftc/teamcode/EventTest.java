package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.core.event.Event;
import org.firstinspires.ftc.teamcode.core.event.EventThread;
import org.firstinspires.ftc.teamcode.core.movement.api.StrafingMovement;
import org.firstinspires.ftc.teamcode.core.movement.impl.StrafedMovementImpl;

@Autonomous
public class EventTest extends LinearOpMode {
    public EventThread eventThread = new EventThread();

    @Override
    public void runOpMode() throws InterruptedException {
        int runTime = 1000;
//        double power = 0.3;
//        int restTime = 1000;

        eventThread.start();
        waitForStart();

        Object object = new Object();
//        StrafingMovement strafedMovement = new StrafedMovementImpl(hardwareMap);
        telemetry.addLine("Doing nothing for " + runTime / 1000 + " second(s).");
        telemetry.update();
//        strafedMovement.drive(power);

        //            strafedMovement.stop();
        eventThread.addEvent(Event.createEventWithMilis(() -> {
            telemetry.addLine("done!");
            object.notifyAll();
        }, runTime));

        synchronized (object) {
            object.wait();
        }
    }
}
