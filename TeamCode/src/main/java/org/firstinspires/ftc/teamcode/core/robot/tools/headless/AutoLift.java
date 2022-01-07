package org.firstinspires.ftc.teamcode.core.robot.tools.headless;

import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.robot.tools.driveop.ControllerLift;
import org.firstinspires.ftc.teamcode.core.thread.EventThread;
import org.firstinspires.ftc.teamcode.core.thread.types.impl.TimedEvent;

import java.util.concurrent.atomic.AtomicBoolean;

import androidx.annotation.NonNull;

public class AutoLift {

    // INTAKING motor 0 , arm 0.76
    // GOING OUT OF DANGER 0.7
    // OUT OF DANGER motor 1375
    // TOP motor 2800 arm 0.3 - 5 1/4
    // MIDDLE motor 1850 arm 0.3 - 5 1/4 from back to rim
    // BOTTOM motor 1375 arm 0.25 - 5 1/4 from back to rim

    protected enum Positions {
        INTAKING,
        SAFE,
        TOP,
        MIDDLE,
        BOTTOM
    }

    private final DcMotor liftMotor;
    private final Servo armServo;
    // private final DigitalChannel bottomSensor;
    private final DigitalChannel topSensor;
    private double curPos = 0;
    private boolean running = false; // currently moving down
    private boolean first = true; // first time its reached bottom
    private final AtomicBoolean dumping = new AtomicBoolean(false);
    private final AtomicBoolean dumpingEventRunning = new AtomicBoolean(false);
    private final EventThread eventThread;
    private TimedEvent event;
    // fix this later
    /**
     * @param eventThread local eventThread instance
     * @param map         local hardwareMap instance
     */
    public AutoLift(EventThread eventThread, @NonNull HardwareMap map) {
        liftMotor = map.get(DcMotor.class,"liftMotor");
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        try { Thread.sleep(100); } catch (InterruptedException ignored) {}
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armServo = map.get(Servo.class,"armServo");
        topSensor = map.get(DigitalChannel.class,"topSensor");
        topSensor.setMode(DigitalChannel.Mode.INPUT);
        this.eventThread = eventThread;


    }
    // TODO make lift aaaaAAAAAAAAAA
}
