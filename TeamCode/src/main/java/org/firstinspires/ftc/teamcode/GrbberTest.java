package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.Variables.servoGrabberThing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name="grib", group = "a")
public class GrbberTest extends DriveMethods{
    Servo grabber;
    public void runOpMode(){
        // LOOK IN VARIABLES FOR GRIBBER POSISITIONS, SEE NUMBER ON GRIBBER
        double clamp = 0.5;
        double release = 0.75;
        int i = 1;
        grabber = hardwareMap.get(Servo.class, "grabber");
        waitForStart();

        while(opModeIsActive()){
            telemetry.addLine("Servo Position: "+ grabber.getPosition());
            telemetry.update();
            if (i<50){
                grabber.setPosition(release);
                telemetry.addLine("Servo Position: "+ grabber.getPosition());
                telemetry.update();
                sleep(3000);
                grabber.setPosition(clamp);
                telemetry.addLine("Servo Position: "+ grabber.getPosition());
                telemetry.update();
                sleep(3000);
                i++;
            }

        }
    }
}
