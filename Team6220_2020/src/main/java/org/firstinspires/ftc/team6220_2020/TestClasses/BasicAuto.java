package org.firstinspires.ftc.team6220_2020.TestClasses;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.team6220_2020.MasterAutonomous;

@Autonomous(name = "Basic Auto", group = "Autonomous")
public class BasicAuto extends MasterAutonomous {

    @Override
    public void runOpMode() {
        Initialize();

        int direction = 0;

        while(gamepad1.x){

            if(gamepad1.right_bumper) {
                direction = 1;
            }else if(gamepad1.left_bumper){
                direction = 2;
            }else if(gamepad1.y){
                direction = 0;
            }
            telemetry.addData("Direction", String.valueOf(direction));
            telemetry.update();
        }

        waitForStart();

        if(direction == 0){
            driveInches(24,90,0.5);
        } else if(direction == 1){
            driveInches(24,0,0.5);
        } else if(direction == 2){
            driveInches(24,270,0.5);
        }

    }
}
