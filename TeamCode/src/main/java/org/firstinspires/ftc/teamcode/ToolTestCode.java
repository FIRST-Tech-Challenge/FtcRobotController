package org.firstinspires.ftc.teamcode;
import android.os.Build;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.parts.Carousel;
import org.firstinspires.ftc.teamcode.parts.ControllerMovement;
import org.firstinspires.ftc.teamcode.parts.Intake;
import org.firstinspires.ftc.teamcode.parts.Lift;

import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.concurrent.TimeUnit;

import androidx.annotation.RequiresApi;

@TeleOp
public class ToolTestCode extends LinearOpMode {
    @RequiresApi(api = Build.VERSION_CODES.O)
    @Override
    public void runOpMode() {
        hardwareMap.get(Blinker.class, "Control Hub");
        hardwareMap.get(Blinker.class, "Expansion Hub 2");
        final GamepadEx moveGamepad = new GamepadEx(gamepad1);
        final GamepadEx toolGamepad = new GamepadEx(gamepad2);
        final Lift lift = new Lift(hardwareMap, toolGamepad, telemetry);
        final ControllerMovement move = new ControllerMovement(hardwareMap,moveGamepad);
        final Carousel carousel = new Carousel(hardwareMap,toolGamepad);
        final Intake intake = new Intake(hardwareMap,toolGamepad);
        final ElapsedTime clocktimer = new ElapsedTime();
        int clocks = 0;
        int clockOutput = 0;
        List<String> list = new LinkedList<>();
        for (HardwareMap.DeviceMapping<? extends HardwareDevice> deviceMapping : hardwareMap.allDeviceMappings) {
            for (Map.Entry<String, ? extends HardwareDevice> entry : deviceMapping.entrySet()) {
                list.add(entry.getKey());
            }
        }
        telemetry.addData("list", String.join(",",list));
        telemetry.update();
        waitForStart();
        telemetry.clearAll();
        while (opModeIsActive()) {
            if (clocktimer.time(TimeUnit.SECONDS) <= 1) { clocks++; }
            else {
                clockOutput = clocks;
                clocks = 0;
                clocktimer.reset();
            }
            telemetry.addData("ClocksPerSecond", clockOutput);
            move.update();
            lift.update();
            carousel.update();
            intake.update();
            telemetry.update();
        }
    }
}
