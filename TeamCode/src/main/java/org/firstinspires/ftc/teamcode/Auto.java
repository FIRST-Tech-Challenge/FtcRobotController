package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.sun.source.util.DocTreePathScanner;
@Autonomous(name = "Auto", preselectTeleOp = "Drive")

public class Auto extends AutoGuts {
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

        //TODO: figure out tick per inch ratio for movement

        //TODO: START SIDEWAYS
        //start
        double greenPixels = pipeline.returnGreen();
        double cyanPixels = pipeline.returnCyan();
        double magentaPixels = pipeline.returnMagenta();
        //first movement
        driveEncoder(-60 * TICKS_PER_INCH , 0, 1);
        turn(.5,45);
        cascadeLift(Level.HIGH);
        driveEncoder(0,3 * TICKS_PER_INCH,1);
        closeClaw(false);
        turn(.5, -45);
        driveEncoder(48 * TICKS_PER_INCH, 0, 1);

        if(greenPixels > cyanPixels && greenPixels > magentaPixels) {
            driveEncoder(0,12,1);
            //park place 3
        }
        if(magentaPixels > cyanPixels && magentaPixels > greenPixels){
            //park place 2
        }
        if(cyanPixels > greenPixels && cyanPixels > magentaPixels){
            driveEncoder(0,-12,1);
            //park place 1
        }

    }


}