package org.firstinspires.ftc.teamcode.opmodes.drive;

import android.os.Build;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;

import org.firstinspires.ftc.teamcode.core.softwaretools.HardwareMapListGenerator;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;
import org.firstinspires.ftc.teamcode.core.robot.tools.driveop.ControllerCarousel;
import org.firstinspires.ftc.teamcode.core.robot.ControllerMovement;
import org.firstinspires.ftc.teamcode.core.robot.tools.driveop.ControllerIntake;
import org.firstinspires.ftc.teamcode.core.robot.tools.driveop.ControllerLift;

import androidx.annotation.RequiresApi;

@TeleOp
@Disabled
public class ToolTestCode extends LinearOpMode {
    public EventThread eventThread = new EventThread(this::opModeIsActive);

    @RequiresApi(api = Build.VERSION_CODES.O)
    @Override
    public synchronized void runOpMode() {
        hardwareMap.get(Blinker.class, "Control Hub");
        hardwareMap.get(Blinker.class, "Expansion Hub 2");
        final GamepadEx moveGamepad = new GamepadEx(gamepad1);
        final GamepadEx toolGamepad = new GamepadEx(gamepad2);
        new ControllerCarousel(eventThread, hardwareMap, toolGamepad);
        HardwareMapListGenerator.gen(hardwareMap, telemetry);
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
        telemetry.clearAll();
        telemetry.update();
        eventThread.start();
        thread.start();
        //noinspection StatementWithEmptyBody
        while (opModeIsActive()) {}
        requestOpModeStop();
    }
}
