package org.firstinspires.ftc.teamcode.Systems;

import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.Hardware.Hardware;
import org.firstinspires.ftc.teamcode.Systems.Mechaisms.Bucket;
import org.firstinspires.ftc.teamcode.Systems.Mechaisms.IntakeSlides;
import org.firstinspires.ftc.teamcode.Systems.Mechaisms.SampleDetector;

public class Intake {

    private IntakeSlides slides;
    private Bucket bucket;
    private SampleDetector detector;

    public enum State {
        TransferReady,
        Deployed,
        Intaking;
    }

    private State currentState;
    private State targetState;

    private double feedRate;

    public Intake(Hardware hardware) {
        slides = new IntakeSlides(hardware);
        bucket = new Bucket(hardware);
        detector = new SampleDetector(hardware);
    }

    public void update() {
        detector.update();
        slides.update();
    }



    public void command() {

    }

    private void targetStateSwitcher(State state) {
        switch (state){
            case TransferReady:
                // Checking Prerequisites
                //boolean bucketPoseTest = ;


                break;
            case Intaking:



                break;
            case Deployed:



                break;
        }
    }

}

