package org.firstinspires.ftc.teamcode.opmodes.drive;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Blinker;

import org.firstinspires.ftc.teamcode.core.robot.ControllerMovement;
import org.firstinspires.ftc.teamcode.core.robot.tools.driveop.ControllerCarousel;
import org.firstinspires.ftc.teamcode.core.robot.tools.driveop.ControllerGrabber;
import org.firstinspires.ftc.teamcode.core.robot.tools.driveop.ControllerIntake;
import org.firstinspires.ftc.teamcode.core.robot.tools.driveop.ControllerLift;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;
import org.firstinspires.ftc.teamcode.core.thread.types.impl.RunWhenOutputChangedOnceEvent;

public class Drive extends LinearOpMode {
    private final EventThread eventThread = new EventThread(this::opModeIsActive);
    @Override
    public void runOpMode() {
        hardwareMap.get(Blinker.class, "Control Hub");
        hardwareMap.get(Blinker.class, "Expansion Hub 2");
        final GamepadEx moveGamepad = new GamepadEx(gamepad1);
        final GamepadEx toolGamepad = new GamepadEx(gamepad2);

        // will automatically run update method
        new ControllerCarousel(eventThread, hardwareMap, toolGamepad);
        new ControllerGrabber(eventThread, hardwareMap, toolGamepad);

        Thread thread = new Thread(() -> {
            final ControllerLift lift = new ControllerLift(eventThread, hardwareMap, toolGamepad, telemetry);
            final ControllerIntake intake = new ControllerIntake(hardwareMap, toolGamepad);
            final ControllerMovement move = new ControllerMovement(hardwareMap,moveGamepad);
            while (opModeIsActive()) {
                move.update();
                intake.update();
                lift.update();
            }
        });
        thread.setPriority(4);
        waitForStart();
        thread.start();
        eventThread.start();
        while (opModeIsActive() && eventThread.isAlive()) {}
        eventThread.interrupt();
        requestOpModeStop();
    }
}
