package org.firstinspires.ftc.teamcode.parts;
import com.arcrobotics.ftclib.gamepad.ButtonReader;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotor;
import androidx.annotation.NonNull;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * lift and arm
 */
public class Lift {
    /**
     * @param map local hardwareMap instance
     * @param telemetry local telemetry instance
     * @param toolGamepad instance of FtcLib GamepadEx
     */
    public Lift(@NonNull HardwareMap map, GamepadEx toolGamepad, Telemetry telemetry) {
        this.liftMotor = map.get(DcMotor.class,"liftMotor");
        this.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        this.liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        try { Thread.sleep(100); } catch (InterruptedException ignored) {}
        this.liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.armServo = map.get(Servo.class,"armServo");
        this.encoderOffset = liftMotor.getCurrentPosition() * -1;
        this.bottomSensor = map.get(DigitalChannel.class,"bottomSensor");
        this.bottomSensor.setMode(DigitalChannel.Mode.INPUT);
        this.topSensor = map.get(DigitalChannel.class,"topSensor");
        this.topSensor.setMode(DigitalChannel.Mode.INPUT);
        this.telemetry = telemetry;
        this.gamepad = toolGamepad;
        this.rBumpReader = new ToggleButtonReader(toolGamepad, GamepadKeys.Button.RIGHT_BUMPER);
        this.aReader = new ButtonReader(toolGamepad, GamepadKeys.Button.A);
    }

    private final Telemetry telemetry;
    private final DcMotor liftMotor;
    private final Servo armServo;
    private final DigitalChannel bottomSensor;
    private final DigitalChannel topSensor;
    private final GamepadEx gamepad;
    private final ToggleButtonReader rBumpReader;
    private final ButtonReader aReader;
    private final double encoderOffset;
    private double curPos = 0;
    private boolean running = false;
    private boolean first = true;

    public void update() {
        final double stickValue = gamepad.getLeftY();
        telemetry.addData("left stick",stickValue);
        telemetry.addData("lift motor power", liftMotor.getPower());
        telemetry.addData("bottomSensor",bottomSensor.getState());
        telemetry.addData("topSensor", topSensor.getState());
        telemetry.addData("motor encoder",curPos);
        telemetry.addData("armServo",armServo.getPosition());
        if (!liftMotor.isBusy()) {
            if (running) {
                liftMotor.setPower(0);
                liftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                running = false;
            }
            if (aReader.wasJustReleased()) { first = true; }
            if ((stickValue >= 0.05 && !topSensor.getState()) || (stickValue <= -0.05 && !bottomSensor.getState())) {
                liftMotor.setPower(stickValue);
            } else {
                if (bottomSensor.getState() && first) {
                    liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    liftMotor.setTargetPosition(0);
                    liftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    liftMotor.setPower(1);
                    running = true;
                    first = false;
                } else { if (topSensor.getState()) {
                        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                    }
                    liftMotor.setPower(0);
                    first = true;
                }
            }
        }
        curPos = liftMotor.getCurrentPosition() + encoderOffset;
        arm();
    }

    private void arm() {
        final double topLiftPosition = 0;
        final double bottomLiftPosition = 0;
        final double loadServoPosition = 0;
        final double liftingServoPosition = 0;
        final double dumpServoPosition = 0;
        if (curPos >= topLiftPosition) {
            rBumpReader.readValue();
            if (rBumpReader.getState()) {
                armServo.setPosition(dumpServoPosition);
            } else {
                armServo.setPosition(liftingServoPosition);
            }
        } else if (curPos >= bottomLiftPosition) {
            armServo.setPosition(liftingServoPosition);
        } else {
            armServo.setPosition(loadServoPosition);
        }
    }
}
