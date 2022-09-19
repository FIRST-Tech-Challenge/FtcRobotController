package org.firstinspires.ftc.forteaching.BasicOpModes;

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.forteaching.SensorCode;

@Disabled
@TeleOp(name = "SensorDemo", group = "demo")
public class SensorDemo extends OpMode {

    private SensorCode sensorCode;
    private DcMotorEx motorL;
    private DcMotorEx motorR;
    private DistanceSensor distanceSensor;
    private ColorSensor colorSensor;
    private TouchSensor touchSensor;
    private Gamepad gamerPad;

    @Override
    public void init() {
        sensorCode = new SensorCode(motorL, motorR, distanceSensor, touchSensor, colorSensor);
    }

    enum State {started, foundColor, isClose, isFinished}

    ;
    State state;

    @Override
    public void start() {
        state = State.started;
        sensorCode.moveForward(1, 5);
    }

    @Override
    public void loop() {
        switch (state) {

            case started:
                if (colorSensor.blue() > 67) {
                    sensorCode.rotateRight(2, 1);
                    state = State.foundColor;
                }
                break;
            case foundColor:
                if (distanceSensor.getDistance(DistanceUnit.INCH) < 1) {
                    sensorCode.rotateRight(2, 1);
                    state = State.isClose;
                }
                break;
            case isClose:
                if (touchSensor.isPressed()) {
                    sensorCode.stop();
                    state = State.isFinished;
                }


        }

    }


}
