package org.firstinspires.ftc.teamcode;
import android.os.Build;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;

import org.firstinspires.ftc.teamcode.core.softwaretools.HardwareMapListGenerator;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;
import org.firstinspires.ftc.teamcode.core.robot.Carousel;
import org.firstinspires.ftc.teamcode.core.robot.ControllerMovement;
import org.firstinspires.ftc.teamcode.core.robot.Intake;
import org.firstinspires.ftc.teamcode.core.robot.Lift;

import androidx.annotation.RequiresApi;

@TeleOp
public class ToolTestCode extends LinearOpMode {
    public EventThread eventThread = new EventThread();
    @RequiresApi(api = Build.VERSION_CODES.O)
    @Override
    public void runOpMode() {
        hardwareMap.get(Blinker.class, "Control Hub");
        hardwareMap.get(Blinker.class, "Expansion Hub 2");
        final GamepadEx moveGamepad = new GamepadEx(gamepad1);
        final GamepadEx toolGamepad = new GamepadEx(gamepad2);
        final Lift lift = new Lift(eventThread, hardwareMap, toolGamepad, telemetry);
        final ControllerMovement move = new ControllerMovement(hardwareMap,moveGamepad);
        new Carousel(eventThread, hardwareMap,toolGamepad);
        new Intake(eventThread, hardwareMap,toolGamepad);
        HardwareMapListGenerator.gen(hardwareMap, telemetry);
        Thread thread = new Thread(() -> {
            while (opModeIsActive()) {
                lift.update();
            }
        });
        Thread thread2 = new Thread(() -> {
            while (opModeIsActive()) {
                move.update();
            }
        });
        thread.setPriority(3);
        thread2.setPriority(3);
        waitForStart();
        telemetry.clearAll();
        eventThread.start();
        thread.start();
        thread2.start();
        //noinspection StatementWithEmptyBody
        while (opModeIsActive()) {}
        eventThread.interrupt();
    }
}
