package org.firstinspires.ftc.teamcode.JackBurr.Servos;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class AxonPickupTest4 extends OpMode {
    public DifferentialV2 diffv2 = new DifferentialV2();
    public DiffConstantsV1 diffConstantsV1 = new DiffConstantsV1();
    public Servo grippers;
    public enum ElbowState {
        DOWN,
        UP
    }
    public enum GripperState {
        OPEN,
        CLOSED
    }
    public ElbowState elbowState = ElbowState.UP;
    public GripperState gripperState = GripperState.OPEN;
    public int GRIPPERS_CLOSED = 1;
    public int GRIPPERS_OPEN = 0;
    public ElapsedTime button_timer = new ElapsedTime();
    public ElapsedTime servoTimer = new ElapsedTime();
    public ElapsedTime elbowTimer = new ElapsedTime();
    @Override
    public void init() {
        diffv2.init(hardwareMap, telemetry);
        grippers = hardwareMap.get(Servo.class, "grippers");
        grippers.setPosition(1);
    }

    @Override
    public void loop() {
        if (gamepad1.x && button_timer.seconds() > 0.3) {
            switch (elbowState) {
                case UP:
                    elbowState = ElbowState.DOWN;
                    break;
                case DOWN:
                    servoTimer.reset();
                    grippers.setPosition(GRIPPERS_CLOSED);
                    gripperState = GripperState.CLOSED;
                    elbowState = ElbowState.UP;
            }
            button_timer.reset();
        }
        if (gripperState == GripperState.OPEN) {
            grippers.setPosition(GRIPPERS_OPEN);
        } else {
            grippers.setPosition(GRIPPERS_CLOSED);
        }
        if (elbowState == ElbowState.UP) {
            if(servoTimer.seconds() > 0.3) {
                if(elbowTimer.seconds() > 1){
                    diffv2.topServosUp();
                }
            }
        }
        else {
            diffv2.topServosDown();
        }
        telemetry.addData("Top Left encoder: ", diffv2.getTopLeftServoEncoderPosition());
        telemetry.addData("Top Right encoder: ", diffv2.getTopRightServoEncoderPosition());
    }
}
