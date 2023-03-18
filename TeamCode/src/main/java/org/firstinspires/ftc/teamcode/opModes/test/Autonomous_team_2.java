package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousLinearModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;


@Autonomous(name="<Autonomous>", group="<OpMode group name>")
public class Autonomous_team_2 extends AutonomousLinearModeBase {


    @Override
    public void run() {
        // Declare class members here
        ElapsedTime runtime = new ElapsedTime();
        //left wheel of forward drive
        Motor motor1 = HardwareMapContainer.motor0;
        //right wheel of forward drive
        Motor motor2 = HardwareMapContainer.motor1;
        //left and right drive
        Motor motor3 = HardwareMapContainer.motor2;
        //Code for autonomous running

        //remember to account for size of robot when inputting coordinates.
        //Can we standardise placing these coordinates on the bottom left corner of the station
        //test data:
        // {{0.5,0},{0.5,0.5}}


        double[][] junction_Coordinates = {{0.5,0},{0.5,0.5}};
        int num_junctions = 3;
        int junction_number = 0;
        double[] next = {junction_Coordinates[junction_number][0],junction_Coordinates[junction_number][1]};
        //Is it possible to use the terminals as a way to check coordinates?

        //remember to account for size of robot when inputting coordinates.
        //Can we standardise placing these coordinates at the center of the terminal
        //test data:
        // {{0,0},{2,2}}
        double[][] terminal = {{0,0},{1,1}};
        //remember to account for size of robot when inputting coordinates.
        //Can we standardise placing these coordinates at the center of the terminal
        //test data:
        //{0,2}
        double[][] cone_coordinates={{0,1}};

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
        // Run until the driver presses STOP
        while (opModeIsActive()) {
            // Code to run in a loop
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //Telling the robot to automatically break from driving loop when 10s left to return to base
            while (time < 20) {
                telemetry.addData("entered first loop", runtime.toString());
                speed1= motor1.getCorrectedVelocity();
                speed2= motor3.getCorrectedVelocity();
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
                    telemetry.addData("Entered cone pick up",current);
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
                    //allowance of 5cm from pole, though subject to change as necessary
                    while (Math.abs(current[0]-closest_coordinate[0])>0.05 && Math.abs(current[1]-closest_coordinate[1])>0.05) {

                        //insert code for proximity sensor

                        speed1 = motor1.getCorrectedVelocity();
                        speed2 = motor3.getCorrectedVelocity();
                        time = runtime.seconds();
                        //finding time elapsed for previous tick
                        elapsed = time - start_elapsed;
                        //resetting the new tick time from this moment
                        start_elapsed=time;
                        telemetry.addData("time", elapsed);
                        //updating current x coordinate
                        current[0]=current[0]+ speed1*elapsed;
                        //updating current y coordinate
                        current[1]=current[1]+ speed2*elapsed;
                        if (current[0]-closest_coordinate[0]>0.05){
                            //getting the robot to decrease the x coordinate
                            motor1.set(-1.0);
                            motor2.set(-1.0);
                        } else if (closest_coordinate[0]-current[0]>0.05){
                            //getting the robot to increase the y coordinate
                            motor1.set(1.0);
                            motor2.set(1.0);
                        } else if (current[1]-closest_coordinate[1]>0.05){
                            //getting the robot to decrease the y coordinate
                            motor3.set(-1.0);
                        } else if (closest_coordinate[1]-current[1]>0.05){
                            //getting the robot to increase the y coordinate
                            motor3.set(1.0);
                        }
                        telemetry.update();
                    }

                    //code for picking up cone

                    has_cone=true;

                    //should we enter code for the robot to drive to the nearest picture in order
                    //to reconfirm coordinates
                } else {
                    telemetry.addData("Entered cone deliver", current);
                    //allowance of 5cm from pole, though subject to change as necessary
                    if (Math.abs(current[0]-next[0])>0.05 && Math.abs(current[1]-next[1])>0.05){
                        if (current[0]-next[0]>0.05){
                            //getting the robot to decrease the x coordinate
                            motor1.set(-1.0);
                            motor2.set(-1.0);
                        } else if (next[0]-current[0]>0.05){
                            //getting the robot to increase the y coordinate
                            motor1.set(1.0);
                            motor2.set(1.0);
                        } else if (current[1]-next[1]>0.05){
                            //getting the robot to decrease the y coordinate
                            motor3.set(-1.0);
                        } else if (next[1]-current[1]>0.05){
                            //getting the robot to increase the y coordinate
                            motor3.set(1.0);
                        }
                        telemetry.addData("next junction", next);
                    } else{
                        //insert code to put down cone

                        if (junction_number>=num_junctions){
                            junction_number=junction_number+1;
                            next[0]=junction_Coordinates[junction_number][0];
                            next[1]=junction_Coordinates[junction_number][1];
                        }
                    }
                }
                telemetry.update();
            }
            while (20 < time && time < 30) {
                telemetry.addData("returning to terminal", current);
                //write code for returning to terminals for the last 10s of the autonomous phase
                speed1= motor1.getCorrectedVelocity();
                speed2= motor3.getCorrectedVelocity();

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
                //allowance of 5cm from terminal, but is subject to change.
                while (Math.abs(current[0]-closest_coordinate[0])<0.05 && Math.abs(current[1]-closest_coordinate[1])<0.05){

                    //insert code for proximity sensor
                    if (current[0]-closest_coordinate[0]>0.05){
                        //getting the robot to decrease the x coordinate
                        motor1.set(-1.0);
                        motor2.set(-1.0);
                    } else if (closest_coordinate[0]-current[0]>0.05){
                        //getting the robot to increase the y coordinate
                        motor1.set(1.0);
                        motor2.set(1.0);
                    } else if (current[1]-closest_coordinate[1]>0.05){
                        //getting the robot to decrease the y coordinate
                        motor3.set(-1.0);
                    } else if (closest_coordinate[1]-current[1]>0.05){
                        //getting the robot to increase the y coordinate
                        motor3.set(1.0);
                    }

                }
                telemetry.update();
            }
        }
    }
}