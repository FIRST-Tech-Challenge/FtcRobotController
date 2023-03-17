package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousLinearModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;

// Test log 15/3/23:
//Figure out why robot can only drive straight when it the velocity reads that both motors are going
//in the opposite direction based on their velocities, implying that the robot cannot turn.
//Perhaphs shift to a turning mechanism where only 1 of the robot's wheels are turning
@Autonomous(name="<Autonomous>", group="<OpMode group name>")
public class TestAutonomousLinearSearch extends AutonomousLinearModeBase {


    @Override
    public void run() {
        // Declare class members here
        ElapsedTime runtime = new ElapsedTime();
        RevIMU internal_measurement_unit;
        //left wheel
        Motor motor1 = HardwareMapContainer.motor0;
        //right wheel
        Motor motor2 = HardwareMapContainer.motor1;
        //Code for autonomous running
        internal_measurement_unit = new RevIMU(HardwareMapContainer.getMap());
        internal_measurement_unit.init();
        //remember to account for size of robot when inputting coordinates.
        //Can we standardise placing these coordinates on the bottom left corner of the station
        //test data:
        // {{0.5,0},{0.5,0.5}}


        double[][] junction_Coordinates = {{0.5,0},{0.5,0.5}};
        int num_junctions = 3;

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
        int junction_number = 0;
        double[] next = {junction_Coordinates[junction_number][0],junction_Coordinates[junction_number][1]};
        waitForStart();
        // Code to run once PLAY is pressed
        boolean has_cone = true;
        runtime.reset();
        double idealheading = 0.00;
        double time = 0.0;
        double elapsed=0.0;
        double start_elapsed = 0.0;
        double speed=0;
        //speed of motor1
        double speed1=0;
        //speed of motor2
        double speed2=0;
        double travelled=0.0;
        double startheading=0;
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
                speed2= motor2.getCorrectedVelocity();
                if (speed1-speed2>-0.2 && speed1-speed2<-0.2){
                    //checking if the robot is turning
                    speed=speed1;
                } else{
                    speed=0;
                }
                //finding total time that has elapsed
                time = runtime.seconds();
                //finding time elapsed for previous tick
                elapsed = time - start_elapsed;
                //resetting the new tick time from this moment
                start_elapsed=time;
                telemetry.addData("time per tick", elapsed);
                //finding distance travelled in last tick
                travelled = speed * elapsed;
                //updating current x coordinate
                current[0] = current[0] + travelled * Math.cos(idealheading);
                //updating current y coordinate
                current[1] = current[1] + travelled * Math.sin(idealheading);
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
                        distance = Math.pow(current[0] - cone_coordinates[index][0], 2);
                        distance = distance + Math.pow(current[1] - cone_coordinates[index][1],2);
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

                        speed = motor1.getCorrectedVelocity();
                        time = runtime.seconds();
                        //finding time elapsed for previous tick
                        elapsed = time - start_elapsed;
                        //resetting the new tick time from this moment
                        start_elapsed=time;
                        telemetry.addData("time", elapsed);
                        travelled = speed * elapsed;
                        //updating current x coordinate
                        current[0] = current[0] + travelled * Math.cos(idealheading);
                        //updating current y coordinate
                        current[1] = current[1] + travelled * Math.sin(idealheading);
                        //finding heading required to drive towards coordinate
                        idealheading = Math.atan2(current[1] - closest_coordinate[1], current[0] - closest_coordinate[0]);
                        startheading = internal_measurement_unit.getHeading();
                        //allowing an error of 1 degree as long as it is constantly updated.
                        if (startheading < idealheading + 1 && startheading > idealheading - 1) {
                            //100 is just a stopgap but need to find ticks per tick of the code so that
                            //the motors stops running the moment another cycle starts
                            motor1.setTargetPosition(100);
                            motor2.setTargetPosition(100);
                        } else if ((startheading - idealheading > 0) || (startheading - idealheading < -180)) {

                            //Cause the motor to turn left
                            //10 degree error allowance for reducing speed is stopgap, need to input degree
                            //change generated by 1 tick of maximum turning
                            if (startheading - idealheading < 10 && startheading - idealheading > -10) {
                                //0.3 is a stopgap
                                motor1.set(-0.3);
                                motor2.set(0.3);
                            }else{
                                motor1.set(-1);
                                motor2.set(1);
                            }
                        } else {
                            //Cause the motor to turn right
                            //10 degree error allowance for reducing speed is stopgap, need to input degree
                            //change generated by 1 tick of maximum turning
                            if (startheading - idealheading < 10 && startheading - idealheading > -10) {
                                //0.3 is a stopgap
                                motor1.set(-0.3);
                                motor2.set(0.3);
                            } else{
                                motor1.set(1);
                                motor2.set(-1);

                            }
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
                        //Calculating heading necessary to go in correct direction
                        idealheading = Math.atan2(current[0] - next[0], current[1] - next[1]);

                        //find a way to calculate the ticks needed to travel this distance

                        // Insert code for proximity sensor

                        startheading = internal_measurement_unit.getHeading();
                        //allowing an error of 1 degree as long as it is constantly updated.
                        if (startheading < idealheading + 1 && startheading > idealheading - 1) {
                            telemetry.addData("driving left wheel", motor1.getCorrectedVelocity());
                            telemetry.addData("driving right wheel",motor2.getCorrectedVelocity());
                            //100 is just a stopgap but need to find ticks per tick of the code so that
                            //the code stops running the moment another cycle starts
                            motor1.setTargetPosition(100);
                            motor2.setTargetPosition(100);

                            //remove this when done
                            //motor1.set(1.0);
                            //motor2.set(-1.0);
                            //apparently the robot does not know how to turn

                        } else if ((startheading - idealheading > 10) || (startheading - idealheading < -180)) {
                            telemetry.addData("turning left", motor1.getCorrectedVelocity());
                            //Cause the motor to turn left
                            //10 degree error allowance for reducing speed is stopgap, need to input degree
                            //change generated by 1 tick of maximum turning
                            if (startheading - idealheading < 10 && startheading - idealheading > -10) {
                                //0.3 is a stopgap
                                motor1.set(-0.3);
                                motor2.set(0.3);
                            } else {
                                motor1.set(-1);
                                motor2.set(1);
                            }

                        } else {
                            telemetry.addData("turning right, right motor", motor2.getCorrectedVelocity());
                            telemetry.addData("turning right, left motor", motor1.getCorrectedVelocity());
                            //Cause the motor to turn right
                            //10 degree error allowance for reducing speed is stopgap, need to input degree
                            //change generated by 1 tick of maximum turning
                            if (startheading - idealheading < 10 && startheading - idealheading > -10) {
                                //0.3 is a stopgap
                                motor1.set(-0.3);
                                motor2.set(0.3);
                            } else {
                                motor1.set(1);
                                motor2.set(-1);
                            }
                        }
                    } else{
                        //code to put down cone on Junction
                        has_cone = false;
                        //incrementing junction number so that we do not put a cone on the same junction
                        junction_number++;
                        if (junction_number<num_junctions) {
                            //redefining junction_Coordinates
                            next[0] = junction_Coordinates[junction_number][0];
                            next[1] = junction_Coordinates[junction_number][1];
                        }
                        telemetry.addData("next junction", next);
                    }
                }
                telemetry.update();
            }
            while (time < 30) {
                telemetry.addData("returning to terminal", current);
                //write code for returning to terminals
                speed1= motor1.getCorrectedVelocity();
                speed2= motor2.getCorrectedVelocity();
                if (speed1-speed2>-0.2 && speed1-speed2<-0.2){
                    //checking if the robot is turning
                    speed=speed1;
                } else{
                    //if the robot is turning, then it technically has not changed position
                    speed=0;
                }
                //finding total time that has elapsed
                time = runtime.seconds();
                //finding time elapsed for previous tick
                elapsed = time - start_elapsed;
                telemetry.addData("time taken", elapsed);
                //resetting the new tick time from this moment
                start_elapsed=time;
                travelled = speed * elapsed;
                //updating current x coordinate
                current[0] = current[0] + travelled * Math.cos(idealheading);
                //updating current y coordinate
                current[1] = current[1] + travelled * Math.sin(idealheading);
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

                    idealheading = Math.atan2(closest_coordinate[1], closest_coordinate[0]);
                    startheading = internal_measurement_unit.getHeading();
                    //allowing an error of 1 degree as long as it is constantly updated.
                    if (startheading < idealheading + 1 && startheading > idealheading - 1) {
                        //100 is just a stopgap but need to find ticks per tick of the code so that
                        //the code stops running the moment another cycle starts

                        motor1.setTargetPosition(100);
                        motor2.setTargetPosition(100);
                    } else if ((startheading - idealheading > 0) || (startheading - idealheading < -180)) {
                        //Cause the motor to turn left
                        //10 degree error allowance for reducing speed is stopgap, need to input degree
                        //change generated by 1 tick of maximum turning
                        if (startheading - idealheading < 10 && startheading - idealheading > -10) {
                            //0.3 is a stopgap
                            motor1.set(-0.3);
                            motor2.set(0.3);
                        } else {
                            motor1.set(-1);
                            motor2.set(1);
                        }

                    } else {
                        //Cause the motor to turn right
                        //10 degree error allowance for reducing speed is stopgap, need to input degree
                        //change generated by 1 tick of maximum turning
                        if (startheading - idealheading < 10 && startheading - idealheading > -10) {
                            //0.3 is a stopgap
                            motor1.set(-0.3);
                            motor2.set(0.3);
                        }else {
                            motor1.set(1);
                            motor2.set(-1);
                        }
                    }
                }
                telemetry.update();
            }
        }
    }
}