package org.firstinspires.ftc.teamcode;

public class Auto extends Hardware {
    @Override
    public void runOpMode(){
        // Fake values for now. Just showing how to call it
        TensorflowDetector recongnizer = new TensorflowDetector(10,20, 1);

        int place = recongnizer.recognizeObjects();

        // Then move to the place.

    }

}
