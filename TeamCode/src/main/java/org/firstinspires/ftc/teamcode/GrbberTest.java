package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.servoGrabberThing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="grib", group = "a")
public class GrbberTest extends DriveMethods{
    Servo grabber;
    public void runOpMode(){
        double clamp = 0.69;
        double release = 0.420;
        int i = 1;
        grabber = hardwareMap.get(Servo.class, "grabber");
        waitForStart();
        while(opModeIsActive()){
            if (i<50){
                grabber.setPosition(release);
                sleep(1000);
                grabber.setPosition(clamp);
                sleep(1000);
                i++;
            }

        }
    }
}
