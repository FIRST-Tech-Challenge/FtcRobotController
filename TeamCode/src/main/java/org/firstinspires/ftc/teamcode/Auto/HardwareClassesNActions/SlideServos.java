package org.firstinspires.ftc.teamcode.Auto.HardwareClassesNActions;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.teleop.Values;

public class SlideServos{
    private Servo intakeSlide1;
    private Servo intakeSlide2;

    public SlideServos(HardwareMap hardwareMap) {
        intakeSlide1 = hardwareMap.get(Servo.class, "5");
        intakeSlide2 = hardwareMap.get(Servo.class,"4");
    }
    public class SlideOut implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeSlide1.setPosition(Values.slide1out);
            intakeSlide2.setPosition(Values.slide2out);
            return false;
        }
    }
    public Action slidesOut() {
        return new SlideOut();
    }

    public class SlideIn implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeSlide1.setPosition(Values.slide1in);
            intakeSlide2.setPosition(Values.slide2in);
            return false;
        }
    }
    public Action slidesIn() {
        return new SlideIn();
    }
    public class SlideWait implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeSlide1.setPosition(Values.slide1wait);
            intakeSlide2.setPosition(Values.slide2wait);
            return false;
        }
    }
    public Action slidesWait() {
        return new SlideWait();
    }

    public class GetOutOfTheWay implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {

            intakeSlide1.setPosition(Values.slide1wait);
            intakeSlide1.setPosition(Values.slide2wait);
            intakeSlide1.setPosition(Values.slide1wait-0.1);
            intakeSlide2.setPosition(Values.slide2wait+0.1);
            return false;
        }
    }
    public Action getOuttaTheWay() {
        return new GetOutOfTheWay();
    }

}