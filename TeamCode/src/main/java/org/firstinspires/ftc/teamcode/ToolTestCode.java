package org.firstinspires.ftc.teamcode;
import android.os.Build;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.thread.event.thread.EventThread;
import org.firstinspires.ftc.teamcode.parts.Carousel;
import org.firstinspires.ftc.teamcode.parts.ControllerMovement;
import org.firstinspires.ftc.teamcode.parts.Intake;
import org.firstinspires.ftc.teamcode.parts.Lift;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

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
        final Carousel carousel = new Carousel(eventThread, hardwareMap,toolGamepad);
        final Intake intake = new Intake(eventThread, hardwareMap,toolGamepad);
        final ElapsedTime clocktimer = new ElapsedTime();
        int clocks = 0;
        int clockOutput = 0;
        List<String> list = new LinkedList<>();
        for (HardwareMap.DeviceMapping<? extends HardwareDevice> deviceMapping : hardwareMap.allDeviceMappings) {
            for (Map.Entry<String, ? extends HardwareDevice> entry : deviceMapping.entrySet()) {
                list.add(entry.getKey());
            }
        }
        telemetry.addData("list", String.join("," ,list));
        telemetry.update();
        waitForStart();
        telemetry.clearAll();
        eventThread.start();
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
        thread.start();
        thread2.start();
        //noinspection StatementWithEmptyBody
        while (opModeIsActive()) {}
    }
}
