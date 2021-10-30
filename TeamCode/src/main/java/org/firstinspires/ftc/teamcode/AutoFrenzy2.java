package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.vision.RGBVisionV1Blue;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

@Autonomous(name="ShitHead3", group="Pushbot")
public class AutoFrenzy2 extends LinearOpMode {
    public static final double dPower = 0.75;

    Hardware robot = new Hardware();   // Use a Pushbot's hardware

    OpenCvCamera phoneCam;

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);
        phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK);
        RGBVisionV1Blue p = new RGBVisionV1Blue(telemetry);
        phoneCam.setPipeline(p);
        phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                /*
                 * Tell the camera to start streaming images to us! Note that you must make sure
                 * the resolution you specify is supported by the camera. If it is not, an exception
                 * will be thrown.
                 *
                 * Also, we specify the rotation that the camera is used in. This is so that the image
                 * from the camera sensor can be rotated such that it is always displayed with the image upright.
                 * For a front facing camera, rotation is defined assuming the user is looking at the screen.
                 * For a rear facing camera or a webcam, rotation is defined assuming the camera is facing
                 * away from the user.
                 */
                phoneCam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });

        waitForStart();

        //Do computer vision here
//THE BELOW CODE IS FOR THE CAROUSEL BEING ON THE LEFT OF THE ROBOT
        int x = 1;
        //p.getPosition(); //get an input of 1,2,3 on where the item is from comp vision
        phoneCam.stopStreaming();
        if(x == 1){
            move(-90,800);
            move(0,500);
            move(90,800);
        }
        else if(x == 2){
            move(90,300);
            move(0, 400);
            move(-90, 300);
        }
        else if(x == 3){
            move(-90,800);
            move(0,500);
            move(90,800);
        }
        //intake
        robot.intake.set(.50);
        move(180,1000);
        sleep(1000);
        robot.intake.set(0);
        //getting to the carousel
        move( 90, 1000); // rotate 90 degrees
        move( 0, 800); //drives to wall
        move(90,1000);//driving parrallel to back wall towards carousel
        move(0,400);
        //spins motor for platform
        robot.carousel.set(.25);
        sleep(1000);
        robot.carousel.set(0);
        //code to drive parallel of the wall towards the parking area
        move(180,200);
        move(-90,2000);
        move(0,500);
        robot.m.driveRobotCentric(0,0,0);

    }
    public void move(double power, double direction, long SLEEP){
        direction = (2*Math.PI*(direction+90)/360);
        robot.m.driveRobotCentric(power*Math.cos(direction),power*Math.sin(direction),0);
        sleep(SLEEP);
        robot.m.driveRobotCentric(0,0,0);
        sleep(100);
    }
    public void move(double direction, long SLEEP){
        move(dPower,direction,SLEEP);
    }
}