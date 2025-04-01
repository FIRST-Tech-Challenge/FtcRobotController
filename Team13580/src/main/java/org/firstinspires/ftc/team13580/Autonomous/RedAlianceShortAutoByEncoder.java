package org.firstinspires.ftc.team13580.Autonomous;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.team13580.RobotHardware;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;


@Autonomous(name="clip", group="Robot")

public class RedAlianceShortAutoByEncoder extends LinearOpMode {
    RobotHardware robot= new RobotHardware(this);
    private ElapsedTime runtime= new ElapsedTime();

    @Override
    public void runOpMode(){
        robot.init();
        double heading;
        waitForStart();
        AprilTagProcessor tag= new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setLensIntrinsics(1130.80338323,1130.80338323, 1280.21111078, 368.731101737 )
                .build();

        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tag)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .build();

        //Closes the claw to secure the specimen
        robot.leftHand.setPosition(1);
        while(opModeIsActive()&& (runtime.seconds()<0.5)){
            telemetry.addData("Path", "Leg1: %41f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(1000);
        //Don't touch this it already works

        //robot.encoderArm(62, 10);
        //robot.upDown.setTargetPosition((int)((73*robot.ARM_TICKS_PER_DEGREE)));

        //go straight for 2 squares
        robot.encoderDrive(0.7,24.0,24.0,24.0,24.0, 15);
        robot.encoderArm(60, 10);
        sleep(50);
        robot.encoderDrive(0.5,8,8,8,8,20);
        sleep(100);
        robot.encoderSpoolie(0.2,10,20);


        //strafe to the left a little bit
        //robot.encoderDrive(0.6,-10.0,10.0,10.0,-10.0, 15);




        //go straight for 2 squares
        //robot.encoderDrive(5.0,8,8,8,8,15);
        sleep(100);
        robot.leftHand.setPosition(0);
        while(opModeIsActive()&& (runtime.seconds()<0.5)){
            telemetry.addData("Path", "Leg1: %41f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(300);
        robot.encoderDrive(3.0,-10,-10,-10,-10,15);
        //sleep(100);
        robot.encoderArm(-70 ,15);
        //robot.encoderDrive(5.0,0.4,0.4,-0.4,-0.4 ,20);




        //robot.encoderArm(-70 ,15);
        robot.encoderDrive(0.8,31.5,-31.5,-31.5,31.5, 15);
        sleep(100);
        robot.encoderDrive(0.8,26.3,26.3,26.3,26.3,15);
        robot.encoderDrive(0.8,4.3,4.3,4.3,4.3,20);
        sleep(100);
        robot.encoderDrive(0.8,12,-12,-12,12, 15);
        sleep(300);
        robot.encoderDrive(0.8,-42.0,-42.0,-42.0,-42.0, 20);
        robot.encoderDrive(0.2,-2,-2,-2,-2,20);
        sleep(300);
        robot.encoderDrive(0.8, 43.0, 43.00, 43.0, 43.0, 15);
        robot.encoderDrive(0.3,3,3,3,3,20);
        sleep(100);
        robot.encoderDrive(0.8, 14.5, -14.5, -14.5, 14.5, 15);
        sleep(100);
        robot.encoderDrive(0.8,-43.0,-43.0,-43.0,-43.0, 15);
        robot.encoderDrive(0.2,-2,-2,-2,-2,20);
        sleep(200);
        robot.encoderDrive(0.8, 17,17,17,17,12);
        sleep(200);
        robot.encoderDrive(0.8, -19,19,19,-19,15);
        sleep(200);
        //the turn why is not going straight into the wall
        robot.encoderDrive(0.8,-45.5,-45.5,45.5,45.5,15);
        robot.encoderDrive(0.9,19.5,19.5,19.5,19.5,15);
        sleep(200);
        robot.encoderArm(17,10);
        robot.leftHand.setPosition(0.4);
        while(opModeIsActive()&& (runtime.seconds()<0.5)){
            telemetry.addData("Path", "Leg1: %41f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(1000);
        robot.encoderDrive(1, -19,-19,-19,-19,15);
        sleep(200);
        robot.encoderDrive(0.9,45,45,-45,-45,15);
        //robot.encoderDrive(0.8,48.5 ,48.5,-48.5,-48.5,15);
        sleep(100);
        robot.encoderDrive(1, -35,35,35,-35,15);
        //robot.encoderDrive(0.7,26.0,26.0,26.0,26.0, 15);
        robot.encoderArm(57, 10);
        sleep(50);
        robot.encoderDrive(0.5,6,6,6,6,20);
        sleep(100);
        robot.encoderSpoolie(0.2,10,20);

        /*
        robot.setDrivePower(0.2,0.2,0.2,0.2);
        runtime.reset();
        while (opModeIsActive()&&(runtime.seconds()<0.00000000001)){
            telemetry.addData("Path", "Leg3: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(1000);
*/
        //makes the elbow go down
        //robot.encoderArm(-25,10);
        /*
        sleep(100);
        //Open claw
        robot.leftHand.setPosition(0);
        while(opModeIsActive()&& (runtime.seconds()<0.5)){
            telemetry.addData("Path", "Leg6: %41f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(100);
        robot.encoderDrive(0.8,-6.5,-6.5,-6.5,-6.5,15);
        robot.encoderArm(-70 ,15);
        robot.encoderDrive(0.8,33.0,-33.0,-33.0,33.0, 15);
        robot.encoderDrive(0.8,24.3,24.3,24.3,24.3,15);

        sleep(200);
        robot.encoderDrive(0.8,12,-12,-12,12, 15);
        // correft before 1st sample
        heading = (int)Math.round(robot.imu.getRobotYawPitchRollAngles().getYaw()/10) * 10;
        if (heading !=    0) {
            robot.setDrivePower(0.3,0.3,-0.3,-0.3);
        }
        while (heading != 0) {
            heading =(int) Math.round(robot.imu.getRobotYawPitchRollAngles().getYaw()/10) * 10;
            telemetry.addData("CORRECTING", "...");
            if (heading == 0){ robot.setDrivePower(0,0,0,0); break;}
        }
        robot.encoderDrive(0.8,-44.0,-44.0,-44.0,-44.0, 20);
        // correct after 1st sample
        heading = (int)Math.round(robot.imu.getRobotYawPitchRollAngles().getYaw()/10) * 10;
        if (heading != 0) {
            robot.setDrivePower(0.3,0.3,-0.3,-0.3);
        }
        while (heading != 0) {
            heading = Math.round(robot.imu.getRobotYawPitchRollAngles().getYaw()/10) * 10;
            telemetry.addData("CORRECTING", "...");
            if (heading == 0){ robot.setDrivePower(0,0,0,0); break;}
        }
        sleep(600);
        robot.encoderDrive(0.8, 47.0, 47.0, 47.0, 47.0, 15);
        sleep(200);
        robot.encoderDrive(0.8, 15.5, -15.5, -15.5, 15.5, 15);
        sleep(300);
        heading = (int)Math.round(robot.imu.getRobotYawPitchRollAngles().getYaw()/10) * 10;

        if (heading != 0) {
            robot.setDrivePower(0.3,0.3,-0.3,-0.3);
        }
        while (heading != 0) {
            heading = Math.round(robot.imu.getRobotYawPitchRollAngles().getYaw()/10) * 10;
            telemetry.addData("CORRECTING", "...");
            if (heading == 0){ robot.setDrivePower(0,0,0,0); break;}
        }

        robot.encoderDrive(0.8,-45.0,-45.0,-45.0,-45.0, 15);
        sleep(400);
        //robot.encoderDrive(0.8, 47.0, 47.0, 47.0, 47.0, 15);
        //sleep(200);
        //robot.encoderDrive(0.8, 11, -11, -11, 11, 15);
        //sleep(400);
        //robot.encoderDrive(0.8, -45.0, -45.0, -45.0, -45.0, 15);
        sleep(200);
        robot.encoderDrive(0.8, 17,17,17,17,12);
        sleep(200);
        heading = (int)Math.round(robot.imu.getRobotYawPitchRollAngles().getYaw()/10) * 10;

        if (heading != 0) {
            robot.setDrivePower(0.3,0.3,-0.3,-0.3);
        }
        while (heading != 0) {
            heading = Math.round(robot.imu.getRobotYawPitchRollAngles().getYaw()/10) * 10;
            telemetry.addData("CORRECTING", "...");
            if (heading == 0){ robot.setDrivePower(0,0,0,0); break;}
        }
        robot.encoderDrive(0.8, -19,19,19,-19,15);
        sleep(200);
        //the turn why is not going straight into the wall
        robot.encoderDrive(0.8,-45.5,-45.5,45.5,45.5,15);
        sleep(200);
        heading = (int)Math.round(robot.imu.getRobotYawPitchRollAngles().getYaw()/10) * 10;

        if (heading != 180) {
            robot.setDrivePower(-0.3,-0.3,0.3,0.3);
        }
        while (heading != 180) {
            heading = Math.round(robot.imu.getRobotYawPitchRollAngles().getYaw()/10) * 10;
            telemetry.addData("CORRECTING", "...");
            if (heading == 180){ robot.setDrivePower(0,0,0,0); break;}
        }

        robot.encoderArm(26,10);
        //AprilTagDetection detect= tag.getDetections().get(0);



        //this is the value that makes the robot approach the wall to pick specimen
        robot.encoderDrive(0.9,50,50,50,50,15);
        sleep(200);


        robot.setHandPositions(-0.8);
        while(opModeIsActive()&& (runtime.seconds()<0.5)){
            telemetry.addData("Path", "Leg1: %41f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(1100);

        robot.encoderArm(37,20);
        robot.encoderDrive(1, -19,-19,-19,-19,15);
        //robot.encoderArm(0,5);
        robot.encoderDrive(0.8,48.5 ,48.5,-48.5,-48.5,15);
        sleep(200);

        //Strafes to the left to place sample
        robot.encoderDrive(1, -50,50,50,-50,15);
        heading =(int) Math.round(robot.imu.getRobotYawPitchRollAngles().getYaw()/10) * 10;
        if (heading != 0) {
            robot.setDrivePower(-0.3,-0.3,0.3,0.3);
        }
        while (heading != 0) {
            heading = Math.round(robot.imu.getRobotYawPitchRollAngles().getYaw()/10) * 10;
            telemetry.addData("CORRECTING", "...");
            if (heading == 0){ robot.setDrivePower(0,0,0,0); break;}
        }
        //makes the arm go up
        robot.encoderArm(20,10);
        robot.encoderDrive(0.8,12.8,12.8,12.8,12.8 ,15);

        sleep(150);
        //makes arm go down
        robot.encoderArm(-30,10);


        //Open claw
        robot.setHandPositions(0.5);
        while(opModeIsActive()&& (runtime.seconds()<0.5)){
            telemetry.addData("Path", "Leg6: %41f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(1000);
        robot.encoderDrive(0.8, 13, -13,-13,13,15);
        robot.setHandPositions(-0.8);
        while(opModeIsActive()&& (runtime.seconds()<0.5)){
            telemetry.addData("Path", "Leg1: %41f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        robot.encoderDrive(0.8, -30,-30,-30,-30,15);
        sleep(100);
        if (heading != 0) {
            robot.setDrivePower(0.3,0.3,-0.3,-0.3);
        }
        while (heading != 0) {
            heading = Math.round(robot.imu.getRobotYawPitchRollAngles().getYaw()/10) * 10;
            telemetry.addData("CORRECTING", "...");
            if (heading == 0){ robot.setDrivePower(0,0,0,0); break;}
        }
        robot.encoderDrive(0.8, 35,-35,-35,35,15);
        robot.encoderDrive(0.8,-12,-12,-12,-12,14);
        */

        //The don't touch has ended, you can now modify the code
        /* Hello, let me show you the basics
        <-the 86 is referred to as line 86 of code so each line has a number
        the code bellow works like this:
        robot.setDrivePower(power of left front wheel, power of left back wheel, power of right front wheel, power of right back wheel);
        comas and semicolons are important do not skip them, you can get an error in your code
        then the while loop has a number in this part runtime.seconds()<0.35<- this number and the ones I explained on top
        are the only thing you can change so if it is something else please ask me first because you can cause an error
         */
        /*
        //strafe to the right a little bit
        robot.setDrivePower(0.5,-0.5,-0.5,0.5);
        runtime.reset();
        while (opModeIsActive()&&(runtime.seconds()<0.7)){
            telemetry.addData("Path", "Leg2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(1000);

        //go straight to get into position
        robot.setDrivePower(0.5,0.5,0.5,0.5);
        runtime.reset();
        while(opModeIsActive()&&(runtime.seconds()<0.3)){
            telemetry.addData("Path","Leg5: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(1000);

        // Strafe to the left
        robot.setDrivePower(0.5,-0.5,-0.5,0.5);
        runtime.reset();
        while (opModeIsActive()&&(runtime.seconds()<0.0000000001)){
            telemetry.addData("Path", "Leg2: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }
        sleep(1000);

        //go back to push
        robot.setDrivePower(-0.5,-0.5,-0.5,-0.5);
        runtime.reset();
        while(opModeIsActive()&&(runtime.seconds()<2.1)){
            telemetry.addData("Path","Leg5: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }


        //go front
        robot.setDrivePower(0.5,0.5,0.5,0.5);
        runtime.reset();
        while(opModeIsActive()&&(runtime.seconds()<2.1)) {
            telemetry.addData("Path", "Leg7: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //strafe to the left
        robot.setDrivePower(0.5,-0.5,-0.5,0.5);
        runtime.reset();
        while(opModeIsActive()&&(runtime.seconds()<0.000001)){
            telemetry.addData("Path","Leg8: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //go back to push
        robot.setDrivePower(-0.5,-0.5,-0.5,-0.5);
        runtime.reset();
        while(opModeIsActive()&&(runtime.seconds()<2.1)){
            telemetry.addData("Path","Leg5: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }


        //go front
        robot.setDrivePower(0.5,0.5,0.5,0.5);
        runtime.reset();
        while(opModeIsActive()&&(runtime.seconds()<2.1)) {
            telemetry.addData("Path", "Leg7: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //strafe to the left
        robot.setDrivePower(0.5,-0.5,-0.5,0.5);
        runtime.reset();
        while(opModeIsActive()&&(runtime.seconds()<0.15)){
            telemetry.addData("Path","Leg8: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }

        //go back to push
        robot.setDrivePower(-0.5,-0.5,-0.5,-0.5);
        runtime.reset();
        while(opModeIsActive()&&(runtime.seconds()<2.1)){
            telemetry.addData("Path","Leg5: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }


        //go front
        robot.setDrivePower(0.5,0.5,0.5,0.5);
        runtime.reset();
        while(opModeIsActive()&&(runtime.seconds()<1.3)) {
            telemetry.addData("Path", "Leg7: %4.1f S Elapsed", runtime.seconds());
            telemetry.update();
        }


        */













    }



    //What are you doing here, please do not touch any code .

    public void setArmPosition(){
        robot.upDown.setTargetPosition((int) (robot.armPosition));

        ((DcMotorEx)robot.upDown).setVelocity(1500);
        robot.upDown.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }




}
