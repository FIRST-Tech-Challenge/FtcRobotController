package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Auton", group="Pushbot")
public class AutonFrenz extends LinearOpMode {

    Hardware robot = new Hardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Autonomous: ", "waiting for start");
        telemetry.update();

        waitForStart();

        //Do computer vision here

        /*
        int x = find(); //get an input of 1,2,3 on where the item is
        if(x == 1){
            move to first box
        }
        else if(x == 2){
            move to second box
        }
        else if(x == 3){
            move to third box
        }
        intake
        move(.75, 'r', 1000); // rotate 90 degrees
        move(.75, 'f', 1000); //drives to wall
        code to drive parallel to the wall
        //spins motor for platform
        robot.m5.set(.25);
        sleep(1000)
        robot.m5.set(0);
        code to drive parallel of the wall towards the parking area
        motorstop();
        */
    }
    public void motorstop(){
        robot.flDrive.set(0);
        robot.blDrive.set(0);
        robot.frDrive.set(0);
        robot.brDrive.set(0);
        sleep(100);
    }
    public void move(double power, char direction, long SLEEP){
        switch (direction){
            case 'b':
                robot.flDrive.set(power);
                robot.blDrive.set(power);
                robot.frDrive.set(power);
                robot.brDrive.set(power);
                sleep(SLEEP);
                break;
            case 'f':
                robot.flDrive.set(-power);
                robot.blDrive.set(-power);
                robot.frDrive.set(-power);
                robot.brDrive.set(-power);
                sleep(SLEEP);
                break;
            case 'r':
                robot.flDrive.set(-power);
                robot.blDrive.set(power);
                robot.frDrive.set(-power);
                robot.brDrive.set(power);
                sleep(SLEEP);
                break;
            case 'l':
                robot.flDrive.set(power);
                robot.blDrive.set(-power);
                robot.frDrive.set(power);
                robot.brDrive.set(-power);
                sleep(SLEEP);
                break;
            case 'x':
                robot.flDrive.set(1);
                robot.blDrive.set(.25);
                robot.frDrive.set(1);
                robot.brDrive.set(.25);
                sleep(SLEEP);
                break;
            case 'y':
                robot.flDrive.set(.25);
                robot.blDrive.set(1);
                robot.frDrive.set(.25);
                robot.brDrive.set(1);
                sleep(SLEEP);
                break;
        }
        motorstop();
    }
}
