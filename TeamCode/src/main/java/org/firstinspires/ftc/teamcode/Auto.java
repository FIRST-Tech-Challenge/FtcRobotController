package org.firstinspires.ftc.teamcode;

public class Auto extends AutoGuts{
    Pipeline pipeline;
    // diagonal fov of camera 55 degrees
    // horizontal fov of camera is 49 degrees
    // vertical fov 28 fov
    // 1280 by 720
    // 16:9 aspect ratio

    @Override
    public void start() {
        double midPointX = pipeline.getRectMidpointX();
        double width = pipeline.getRectWidth();
        int height = pipeline.getRectHeight();
        double area = pipeline.getRectArea();
        double turn = 0;
        double powerGroup1 = 0;
        double powerGroup2 = 0;
        double speed = 0;
        double angleRatio = 0;
        double angle = 0;
        double loopCounter = 0;

        try {
            driveEncoder(0, 100, 1);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
        turn(1,90);
        do{
            area = pipeline.getRectArea();
            width = pipeline.getRectWidth();
            midPointX = pipeline.getRectMidpointX();
            height = pipeline.getRectHeight();

            cascadeMoveClaw(height - 100, 1, 1, true);

        }
        while(area > 200);





    }


}

