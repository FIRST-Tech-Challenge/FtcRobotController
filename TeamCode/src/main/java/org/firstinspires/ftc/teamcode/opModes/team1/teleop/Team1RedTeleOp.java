package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousLinearModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;


@Autonomous(name="<Team_2_autonomous>", group="<OpMode group name>")
public class Team1RedTeleOp extends AutonomousLinearModeBase {


    @Override
    public void run() {

        // Declare class members here

        ElapsedTime runtime = new ElapsedTime();
        //left wheel of forward drive (note that this actually goes in reverse)
        Motor motor1 = HardwareMapContainer.motor0;
        //right wheel of forward drive
        Motor motor2 = HardwareMapContainer.motor1;
        //left and right drive
        Motor motor3 = HardwareMapContainer.motor2;
        //left arm
        Motor left_arm = HardwareMapContainer.motor2;
        //right arm
        Motor right_arm = HardwareMapContainer.motor3;
        Servo left_intake=HardwareMapContainer.getServo(0);
        Servo right_intake=HardwareMapContainer.getServo(1);

        //Code for autonomous running

        //remember to account for size of robot when inputting coordinates.
        //Can we standardise placing these coordinates on the bottom left corner of the station
        //test data:
        // {{500,0},{500,500}}


        double[][] junction_Coordinates = {{500,0},{0.5,500}};
        int num_junctions = 2;
        int junction_number = 0;
        double[] next = {junction_Coordinates[junction_number][0],junction_Coordinates[junction_number][1]};
        //Is it possible to use the terminals as a way to check coordinates?

        //remember to account for size of robot when inputting coordinates.
        //Can we standardise placing these coordinates at the center of the terminal
        //test data:
        // {{0,0},{1000,1000}}
        double[][] terminal = {{0,0},{1000,1000}};
        //remember to account for size of robot when inputting coordinates.
        //Can we standardise placing these coordinates at the center of the terminal
        //test data:
        //{0,1000}
        double[][] cone_coordinates={{0,1000},{5000,5000}};

        //used later for finding closest cone stack
        double[] closest_coordinate= {cone_coordinates[0][0], cone_coordinates[0][1]};
        double distance=0.0;
        double smallest_distance = 0.0;

        double[] current = {0, 0};
        waitForStart();
        // Code to run once PLAY is pressed
        boolean has_cone = true;
        runtime.reset();
        double time = 0.0;
        double elapsed=0.0;
        double start_elapsed = 0.0;
        //speed of forward drive
        double speed1=0;
        //speed of left/right drive
        double speed2=0;
        motor1.resetEncoder();
        motor2.resetEncoder();
        motor3.resetEncoder();
        double start_time=0.0;
        // Run until the driver presses STOP
        while (opModeIsActive()) {
            // Code to run in a loop
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //Telling the robot to automatically break from driving loop when 10s left to return to base
            while (time < 20) {
                telemetry.addData("entered first loop", runtime.toString());
                //remember that the speed is in rpm, and the gear ratio of motor to the wheel is 15:20
                //3:4 gear ratio and the wheel is 85mm in diameter, which should equate to 267.04
                //how did I get 1.4167 mm per turn?
                //motor 1 goes in reverse so I need to use motor 2's speed instead
                speed1= motor2.getCorrectedVelocity()*0.75*267.04/60;
                speed2= motor3.getCorrectedVelocity()*0.75*267.04/60;
                //finding total time that has elapsed
                time = runtime.seconds();
                //finding time elapsed for previous tick
                elapsed = time - start_elapsed;
                //resetting the new tick time from this moment
                start_elapsed=time;
                telemetry.addData("time per tick", elapsed);
                //finding distance travelled in last tick
                current[0]=current[0]+ speed1*elapsed;
                current[1]=current[1]+ speed2*elapsed;
                //Distance from target in terms of x distance and y distance
                if (has_cone == false) {
                    //Run code to return to nearest loading station
                    //finding distance from each robot to cones using pythagoras
                    //assuming that the first coordinate is the closest
                    smallest_distance = Math.pow(current[0] - cone_coordinates[0][0], 2);
                    smallest_distance = smallest_distance + Math.pow(current[1] - cone_coordinates[0][1],2);
                    //running through all the cone coordinates to find the coordinate with the
                    //lowest distance
                    for (int index = 1;index<cone_coordinates.length; index++){
                        //finding distance using pythagoras
                        distance = Math.pow(cone_coordinates[index][0]- current[0], 2);
                        distance = distance + Math.pow(cone_coordinates[index][1]-current[1],2);
                        if (distance<smallest_distance){
                            //updating closest cones if the distance is smaller than the current "
                            //closest coordinate
                            smallest_distance=distance;
                            closest_coordinate=cone_coordinates[index];
                        }
                    }
                    //getting the robot to drive towards the cones
                    //allowance of 1cm from pole, though subject to change as necessary
                    while (Math.abs(current[0]-closest_coordinate[0])>10 && Math.abs(current[1]-closest_coordinate[1])>10) {
                        speed1 = motor2.getCorrectedVelocity()*0.75*267.04/60;
                        speed2 = motor3.getCorrectedVelocity()*0.75*267.04/60;
                        time = runtime.seconds();
                        //finding time elapsed for previous tick
                        elapsed = time - start_elapsed;
                        //resetting the new tick time from this moment
                        start_elapsed=time;
                        telemetry.addData("time", elapsed);
                        telemetry.addData("Entered cone pick up", "x_coordinate" +current[0]);
                        telemetry.addData("Entered cone pick up", "y_coordinate" +current[1]);
                        //updating current x coordinate
                        current[0]=current[0]+ speed1*elapsed;
                        //updating current y coordinate
                        current[1]=current[1]+ speed2*elapsed;
                        //allowing an error of 1cm
                        if (current[0]-closest_coordinate[0]>10){
                            //making sure that if motor 3 was running, it would stop
                            motor3.set(0);
                            //getting the robot to decrease the x coordinate
                            //reduce speed if within 20cm of target
                            if (current[0]-closest_coordinate[0]<200) {
                                motor1.set(0.3);
                                motor2.set(-0.3);
                            } else{
                                motor1.set(0.8);
                                motor2.set(-0.8);
                            }
                        } else if (closest_coordinate[0]-current[0]>10){
                            //making sure that if motor 3 was running, it would stop
                            motor3.set(0);
                            //getting the robot to increase the y coordinate
                            //reduce speed if within 20cm of target
                            if (closest_coordinate[0]-current[0]<200) {
                                motor1.set(-0.3);
                                motor2.set(0.3);
                            } else{
                                motor1.set(-0.8);
                                motor2.set(0.8);
                            }
                        } else if (current[1]-closest_coordinate[1]>10){
                            //making sure that if motor1 and motor2 was running, it would stop
                            motor1.set(0);
                            motor2.set(0);
                            //getting the robot to decrease the y coordinate
                            //reduce speed if within 20cm of target
                            if (current[1]-closest_coordinate[1]<200) {
                                motor3.set(-0.3);
                            } else{
                                motor3.set(-0.8);
                            }
                        } else if (closest_coordinate[1]-current[1]>10){
                            //making sure that if motor1 and motor2 was running, it would stop
                            motor1.set(0);
                            motor2.set(0);
                            //getting the robot to increase the y coordinate
                            //reduce speed if within 20cm of target
                            if (closest_coordinate[1]-current[1]<200) {
                                motor3.set(0.3);
                            } else{
                                motor3.set(0.8);
                            }
                        }
                        telemetry.update();
                    }
                    //getting the robot to stop
                    motor1.set(0);
                    motor2.set(0);
                    motor3.set(0);
                    //code for picking up cone
                    start_time=runtime.seconds();
                    //get the robot to close the servo
                    //Assuming position 0 is leftmost
                    left_intake.setPosition(1);
                    right_intake.setPosition(0);
                    //get the robot to lift the cone for 0.5s
                    while (runtime.seconds()-start_time<0.5) {
                        left_arm.set(-1);
                        right_arm.set(-1);
                    }
                    has_cone=true;

                    //should we enter code for the robot to drive to the nearest picture in order
                    //to reconfirm coordinates
                } else {
                    telemetry.addData("Entered cone deliver", "x_coordinate" +current[0]);
                    telemetry.addData("Entered cone deliver", "y_coordinate" +current[1]);
                    //allowance of 1cm from pole, though subject to change as necessary
                    if (Math.abs(current[0]-next[0])>10 && Math.abs(current[1]-next[1])>10){
                        if (current[0]-next[0]>10){
                            //making sure that if motor 3 was running, it would stop
                            motor3.set(0);
                            //getting the robot to decrease the x coordinate
                            //reduce speed if within 20cm of target
                            if (current[0]-next[0]<200) {
                                motor1.set(0.3);
                                motor2.set(-0.3);
                            } else{
                                motor1.set(0.8);
                                motor2.set(-0.8);
                            }
                        } else if (next[0]-current[0]>10){
                            //making sure that if motor 3 was running, it would stop
                            motor3.set(0);
                            //getting the robot to increase the y coordinate
                            //reduce speed if within 20cm of target
                            if (next[0]-current[0]<200) {
                                motor1.set(-0.3);
                                motor2.set(0.3);
                            } else{
                                motor1.set(-0.8);
                                motor2.set(0.8);
                            }
                        } else if (current[1]-next[1]>10){
                            //making sure that if motor1 and motor2 was running, it would stop
                            motor1.set(0);
                            motor2.set(0);
                            //getting the robot to decrease the y coordinate
                            //reduce speed if within 20cm of target
                            if (current[1]-next[1]<200) {
                                motor3.set(-0.3);
                            } else{
                                motor3.set(-0.8);
                            }
                        } else if (next[1]-current[1]>10){
                            //making sure that if motor1 and motor2 was running, it would stop
                            motor1.set(0);
                            motor2.set(0);
                            //getting the robot to increase the y coordinate
                            //reduce speed if within 20cm of target
                            if (next[1]-current[1]<200) {
                                motor3.set(0.3);
                            } else{
                                motor3.set(0.8);
                            }
                        }
                        telemetry.addData("next junction", next);
                    } else{
                        //getting the robot to stop
                        motor1.set(0);
                        motor2.set(0);
                        motor3.set(0);
                        //insert code to put down cone
                        start_time=runtime.seconds();
                        //get the robot to lower the cone for 0.5s
                        while (runtime.seconds()-start_time<0.5) {
                            left_arm.set(1);
                            right_arm.set(1);
                        }
                        //get the robot to open the intake
                        //Assuming position 0 is leftmost
                        left_intake.setPosition(0);
                        right_intake.setPosition(1);
                        if (junction_number<num_junctions){
                            junction_number=junction_number+1;
                            next[0]=junction_Coordinates[junction_number][0];
                            next[1]=junction_Coordinates[junction_number][1];
                        }
                    }
                }
                telemetry.update();
            }
            while (20 < time && time < 30) {
                //get the robot to close the intake
                //Assuming position 0 is leftmost
                left_intake.setPosition(1);
                right_intake.setPosition(0);
                telemetry.addData("returning to terminal", "x_coordinate" +current[0]);
                telemetry.addData("returning to terminal", "y_coordinate"+ current[1]);
                //write code for returning to terminals for the last 10s of the autonomous phase
                speed1= motor2.getCorrectedVelocity()*0.75*267.04/60;
                speed2= motor3.getCorrectedVelocity()*0.75*267.04/60;
                //finding total time that has elapsed
                time = runtime.seconds();
                //finding time elapsed for previous tick
                elapsed = time - start_elapsed;
                telemetry.addData("time taken", elapsed);
                //resetting the new tick time from this moment
                start_elapsed=time;
                current[0] = current[0] + speed1*elapsed;
                //updating current y coordinate
                current[1] = current[1] + speed2*elapsed;
                //redefining closest terminal as the first terminal in list
                closest_coordinate[0]=terminal[0][0];
                closest_coordinate[1]=terminal[0][1];
                //finding distance from each terminal to find most efficient path to terminal
                //using pythagoras
                smallest_distance = Math.pow(current[0] - terminal[0][0], 2);
                smallest_distance = smallest_distance + Math.pow(current[1] - terminal[0][1],2);
                //running through all the terminal coordinates to find the coordinate with the
                //lowest distance
                for (int index = 1;index<terminal.length; index++){
                    //finding distance using pythagoras
                    distance = Math.pow(current[0] - terminal[index][0], 2);
                    distance = distance + Math.pow(current[1] - terminal[index][1],2);
                    if (distance<smallest_distance){
                        //updating closest cones if the distance is smaller than the current "
                        //closest coordinate
                        smallest_distance=distance;
                        closest_coordinate=cone_coordinates[index];
                    }
                }
                //allowance of 1cm from terminal, but is subject to change.
                while (Math.abs(current[0]-closest_coordinate[0])<10 && Math.abs(current[1]-closest_coordinate[1])<10){
                    if (current[0]-closest_coordinate[0]>10){
                        //making sure that if motor3 was running, it would stop
                        motor3.set(0);
                        //getting the robot to decrease the x coordinate
                        //reduce speed if within 50cm of target
                        if (current[0]-closest_coordinate[0]<200) {
                            motor1.set(0.3);
                            motor2.set(-0.3);
                        } else{
                            motor1.set(0.8);
                            motor2.set(-0.8);
                        }
                    } else if (closest_coordinate[0]-current[0]>10){
                        //making sure that if motor 3 was running, it would stop
                        motor3.set(0);
                        //getting the robot to increase the y coordinate
                        //reduce speed if within 20cm of target
                        if (closest_coordinate[0]-current[0]<200) {
                            motor1.set(-0.3);
                            motor2.set(0.3);
                        } else{
                            motor1.set(-0.8);
                            motor2.set(0.8);
                        }
                    } else if (current[1]-closest_coordinate[1]>10){
                        //making sure that if motor1 and motor2 was running, it would stop
                        motor1.set(0);
                        motor2.set(0);
                        //getting the robot to decrease the y coordinate
                        //reduce speed if within 20cm of target
                        if (current[1]-closest_coordinate[1]<200) {
                            motor3.set(-0.3);
                        } else{
                            motor3.set(-0.8);
                        }
                    } else if (closest_coordinate[1]-current[1]>10){
                        //making sure that if motor1 and motor2 was running, it would stop
                        motor1.set(0);
                        motor2.set(0);
                        //getting the robot to increase the y coordinate
                        //reduce speed if within 20cm of target
                        if (closest_coordinate[1]-current[1]<200) {
                            motor3.set(0.3);
                        } else{
                            motor3.set(0.8);
                        }
                    }
                    telemetry.update();
                }
                //getting the robot to stop
                motor1.set(0);
                motor2.set(0);
                motor3.set(0);
            }
        }
    }
}