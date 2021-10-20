package org.firstinspires.ftc.teamcode.tools;
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
    public Lift(@NonNull HardwareMap map, Telemetry telemetry, GamepadEx toolGamepad) {
        this.liftMotor = map.get(DcMotor.class,"liftMotor");
        this.liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE)
        this.armServo = map.get(Servo.class,"armServo");
        this.encoderOffset = liftMotor.getCurrentPosition() * -1;
        this.bottomSensor = map.get(DigitalChannel.class,"bottomSensor");
        this.bottomSensor.setMode(DigitalChannel.Mode.INPUT);
        this.topSensor = map.get(DigitalChannel.class,"topSensor");
        this.topSensor.setMode(DigitalChannel.Mode.INPUT);
        this.telemetry = telemetry;
        this.gamepad = toolGamepad;
        this.rBumpReader = new ToggleButtonReader(toolGamepad, GamepadKeys.Button.RIGHT_BUMPER);
    }

    private final Telemetry telemetry;
    private final DcMotor liftMotor;
    private final Servo armServo;
    private final DigitalChannel bottomSensor;
    private final DigitalChannel topSensor;
    private final GamepadEx gamepad;
    private final ToggleButtonReader rBumpReader;
    private final double encoderOffset;
    private double curPos = 0;

    public void update() {
        final double afloatValue = 0.05;
        final double stickValue = gamepad.getLeftY();
        telemetry.addData("left stick",stickValue);
        telemetry.addData("lift motor power", liftMotor.getPower());
        telemetry.addData("bottomSensor",bottomSensor.getState());
        telemetry.addData("topSensor", topSensor.getState());
        telemetry.addData("motor encoder",curPos);
        telemetry.addData("armServo",armServo.getPosition());
        if ((stickValue >= 0.05 && !topSensor.getState()) || (stickValue <= -0.05 && !bottomSensor.getState())) {
            liftMotor.setPower(stickValue);
        } else {
            liftMotor.setPower(0);
        }
        curPos = liftMotor.getCurrentPosition() + encoderOffset;
        arm();
    }

    private void arm() {
        double topLiftPosition = 0;
        double bottomLiftPosition = 0;
        double loadServoPosition = 0;
        double liftingServoPosition = 0;
        double dumpServoPosition = 0;
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
