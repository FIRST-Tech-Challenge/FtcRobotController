package org.firstinspires.ftc.teamcode.opmodes.drive;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.core.robot.ControllerMovement;
import org.firstinspires.ftc.teamcode.core.robot.tools.driveop.ControllerCarousel;
import org.firstinspires.ftc.teamcode.core.robot.tools.driveop.ControllerGrabber;
import org.firstinspires.ftc.teamcode.core.robot.tools.driveop.ControllerIntake;
import org.firstinspires.ftc.teamcode.core.robot.tools.driveop.ControllerLift;
import org.firstinspires.ftc.teamcode.core.thread.old.EventThread;

@TeleOp
@Disabled
public class Drive extends LinearOpMode {
    private void waitTilDone() {
        //noinspection StatementWithEmptyBody
        while (opModeIsActive() && eventThread.isAlive()) {}
    }

    private final EventThread eventThread = new EventThread(() -> !isStopRequested());



    @Override
    public void runOpMode() {
        hardwareMap.get(Blinker.class, "Control Hub");
        hardwareMap.get(Blinker.class, "Expansion Hub 2");
        final GamepadEx moveGamepad = new GamepadEx(gamepad1);
        final GamepadEx toolGamepad = new GamepadEx(gamepad2);

        // will automatically run update method
        new ControllerCarousel(eventThread, hardwareMap, toolGamepad, 1);
        final ControllerGrabber grabber = new ControllerGrabber(eventThread, hardwareMap, toolGamepad);

        Thread thread = new Thread(() -> {
            final ControllerLift lift = new ControllerLift(eventThread, hardwareMap, toolGamepad, grabber);
            final ControllerIntake intake = new ControllerIntake(hardwareMap, eventThread, toolGamepad, true);
            final ControllerMovement move = new ControllerMovement(hardwareMap,moveGamepad);
            while (opModeIsActive()) {
                move.update();
                intake.update(lift.getPosition());
                lift.update();
            }
            lift.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        });
        thread.setPriority(4);
        waitForStart();
        thread.start();
        grabber.init();
        eventThread.start();
        waitTilDone();
        eventThread.interrupt();
        requestOpModeStop();
    }
}
