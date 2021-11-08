package org.firstinspires.ftc.teamcode.parts;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import androidx.annotation.NonNull;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.core.thread.event.thread.EventThread;
import org.firstinspires.ftc.teamcode.core.thread.event.types.impl.TimedEvent;
import com.qualcomm.robotcore.hardware.Servo;
import java.util.concurrent.atomic.AtomicBoolean;

/**
 * lift and arm
 */
public class Lift {
    /**
     * @param map local hardwareMap instance
     * @param telemetry local telemetry instance
     * @param toolGamepad instance of FtcLib GamepadEx
     */
    public Lift(EventThread eventThread, @NonNull HardwareMap map, GamepadEx toolGamepad, Telemetry telemetry) {
        liftMotor = map.get(DcMotor.class,"liftMotor");
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        try { Thread.sleep(100); } catch (InterruptedException ignored) {}
        liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        armServo = map.get(Servo.class,"armServo");
        encoderOffset = liftMotor.getCurrentPosition() * -1;
        // bottomSensor = map.get(DigitalChannel.class,"bottomSensor");
        // bottomSensor.setMode(DigitalChannel.Mode.INPUT);
        topSensor = map.get(DigitalChannel.class,"topSensor");
        topSensor.setMode(DigitalChannel.Mode.INPUT);
        this.telemetry = telemetry;
        gamepad = toolGamepad;
        rBumpReader = new ButtonReader(toolGamepad, GamepadKeys.Button.RIGHT_BUMPER);
        aReader = new ButtonReader(toolGamepad, GamepadKeys.Button.A);
        this.eventThread = eventThread;
    }
    private final Telemetry telemetry;
    private final DcMotor liftMotor;
    private final Servo armServo;
    // private final DigitalChannel bottomSensor;
    private final DigitalChannel topSensor;
    private final GamepadEx gamepad;
    private final ButtonReader rBumpReader;
    private final ButtonReader aReader;
    private final double encoderOffset;
    private double curPos = 0;
    private boolean running = false; // currently moving down
    private boolean first = true; // first time its reached bottom
    private final AtomicBoolean dumping = new AtomicBoolean(false);
    private final AtomicBoolean dumpingEventRunning = new AtomicBoolean(false);
    private final EventThread eventThread;
    public void update() {
        final double stickValue = gamepad.getLeftY();
        telemetry.addData("left stick",stickValue);
        telemetry.addData("lift motor power", liftMotor.getPower());
        telemetry.addData("lift motor state",liftMotor.getMode());
        telemetry.addData("bottomSensor",curPos <= 10);
        telemetry.addData("motor encoder",curPos);
        telemetry.addData("topSensor", topSensor.getState());
        telemetry.addData("armServo",armServo.getPosition());
        if ((liftMotor.getMode() != DcMotor.RunMode.RUN_TO_POSITION || !liftMotor.isBusy()) && !dumpingEventRunning.get()) {
            if (running) {
                liftMotor.setPower(0);
                liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                running = false;
            }
            aReader.readValue();
            if (aReader.wasJustReleased() && armServo.getPosition() > 0.1) {
                first = true;
                curPos = 10;
            }
            if ((stickValue >= 0.05 && !topSensor.getState()) || (stickValue <= -0.05 && curPos >= 10)) {
                liftMotor.setPower(stickValue);
            } else {
                if (curPos <= 10 && first) {
                    liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    liftMotor.setTargetPosition(0);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftMotor.setPower(1);
                    running = true;
                    first = false;
                } else {
                    if (topSensor.getState()) {
                        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    } else if (curPos >= 10) {
                        first = true;
                    }
                    liftMotor.setPower(0);
                }
            }
        }
        curPos = liftMotor.getCurrentPosition() + encoderOffset;
        arm();
    }
    private void arm() {
        rBumpReader.readValue();
        if (curPos >= (double) 960) { // completely away from intake interference
            if (rBumpReader.wasJustReleased()) {
                dumping.set(!dumping.get());
            }
            if (dumping.get()) {
                if (!dumpingEventRunning.get()) {
                    armServo.setPosition(0);
                    eventThread.addEvent(new TimedEvent(() -> {
                        dumping.set(false);
                        dumpingEventRunning.set(false);
                    }, 800));
                }
            } else {
                armServo.setPosition(0.7);
            }
        } else if (curPos >= (double) 50) { // if it is away from load zone but still interfering
            armServo.setPosition(0.847); // in between load and lift pos
        } else { // in load zone
            armServo.setPosition(0.88); // load pos
        }
    }
}
