package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.constants.FConstants;
import org.firstinspires.ftc.teamcode.pedroPathing.constants.LConstants;

public class Teleop extends OpMode {

    //Pedro Pathing Follower
    Follower follower;



    @Override
    public void init() {
        //ONE TIME INIT CALL
        //This is where you will call all of the constructors for your different subsystems.
        //Eventually, once I have them setup, you will see examples like intake, outtake, etc.
        //Telemetry is also reinitialized here as to allow you to view it on FTC Dashboard (http://192.168.43.1:8080/dash)
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        follower = new Follower(hardwareMap, FConstants.class, LConstants.class);

    }




    @Override
    public void init_loop() {
        //INIT LOOP
        //We never put anything here for motors/servos
        //as its against the rules for the bots to move AT ALL
        //in the initialization phase of Driver Controlled.
        //For testing purposes, obviously yes you can put stuff here.


        telemetry.addLine("Initialized!");
        telemetry.update();
    }

    @Override
    public void start(){
        //ONE TIME DRIVER PERIOD CALL
        //something like starting positions :)
    }

    @Override
    public void loop() {
        //DRIVER LOOP
        //We will only ever use the LOGITECH CONTROLLER SCHEME
        //https://gm0.org/en/latest/docs/software/tutorials/gamepad.html


        //Gamepad 1
        if(gamepad1.a){
            telemetry.addLine("BUTTON A PRESSED");
        }
        if(gamepad1.b){
            telemetry.addLine("BUTTON B PRESSED");

        }
        if(gamepad1.x){
            telemetry.addLine("BUTTON X PRESSED");

        }
        if(gamepad1.y){
            telemetry.addLine("BUTTON Y PRESSED");

        }

        if(gamepad1.dpad_down){
            telemetry.addLine("BUTTON DPAD DOWN PRESSED");
        }
        if(gamepad1.dpad_right){
            telemetry.addLine("BUTTON DPAD RIGHT PRESSED");

        }
        if(gamepad1.dpad_left){
            telemetry.addLine("BUTTON DPAD LEFT PRESSED");

        }
        if(gamepad1.dpad_up){
            telemetry.addLine("BUTTON DPAD UP PRESSED");

        }






        //Gamepad 2
        if(gamepad2.a){
            telemetry.addLine("BUTTON A PRESSED");
        }
        if(gamepad2.b){
            telemetry.addLine("BUTTON B PRESSED");

        }
        if(gamepad2.x){
            telemetry.addLine("BUTTON X PRESSED");

        }
        if(gamepad2.y){
            telemetry.addLine("BUTTON Y PRESSED");

        }

        if(gamepad2.dpad_down){
            telemetry.addLine("BUTTON DPAD DOWN PRESSED");
        }
        if(gamepad2.dpad_right){
            telemetry.addLine("BUTTON DPAD RIGHT PRESSED");

        }
        if(gamepad2.dpad_left){
            telemetry.addLine("BUTTON DPAD LEFT PRESSED");

        }
        if(gamepad2.dpad_up){
            telemetry.addLine("BUTTON DPAD UP PRESSED");

        }









        //Using the gamepad1 sticks, we set the different VECTORS that the drive will try to follow.
        follower.setTeleOpMovementVectors(-gamepad1.left_stick_y, -gamepad1.left_stick_x, -gamepad1.right_stick_x, false);
        //Updates all aspects of the follower, including but not limited to its
        //Current pose data, power vectors, target poses, etc.
        follower.update();

        //Adding telemetry so that way you can monitor things that happen in a OpMode.
        //Telemetry is especially useful for things like motors so you can actively view the encoder positions.
        telemetry.addData("X", follower.getPose().getX());
        telemetry.addData("Y", follower.getPose().getY());
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.getPose().getHeading()));

        //Sends the 'packet' of telemetry to both the Driver Hub and FTCDashboard
        telemetry.update();





    }
}
