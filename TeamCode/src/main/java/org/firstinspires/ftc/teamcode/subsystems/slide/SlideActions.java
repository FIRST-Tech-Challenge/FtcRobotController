package org.firstinspires.ftc.teamcode.subsystems.slide;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import java.security.PrivilegedAction;

public class SlideActions {
    private Servo left;
    private Servo right;

    public SlideActions(HardwareMap hardwareMap) {
        left = hardwareMap.servo.get("slideLeftServo");
        right = hardwareMap.servo.get("slideServoRight");
    }

    private class SlideUp implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double pos = 1;
            left.setPosition(pos);
            right.setPosition(pos);
            return left.getPosition() != pos && right.getPosition() != pos;
        }
    }

    public Action SlideUp() {
        return new SlideUp();
    }

    private class SlideDown implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            double pos = 0;
            left.setPosition(pos);
            right.setPosition(pos);
            return left.getPosition() != pos && right.getPosition() != pos;
        }
    }

    public Action SlideDown() {
        return new SlideDown();
    }
}
