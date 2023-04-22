package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousLinearModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;


@Autonomous(name="<Team_2_autonomous_red>", group="<OpMode group name>")
public class Autonomous_team_2 extends AutonomousLinearModeBase {
    //This code assumes that motor 1 / the left motor goes in reverse
    @Override
    public void run() {
        // Declare class members here
        ElapsedTime runtime = new ElapsedTime();
        RevIMU internal_measurement_unit;
        //left wheel
        Motor motor1 = HardwareMapContainer.motor0;
        //right wheel
        Motor motor2 = HardwareMapContainer.motor1;
        //left arm
        Motor left_arm = HardwareMapContainer.motor2;
        //right arm
        Motor right_arm = HardwareMapContainer.motor3;
        Servo left_intake = null; //HardwareMapContainer.getServo(0);
        Servo right_intake = null;//HardwareMapContainer.getServo(1);
        //Code for autonomous running
        internal_measurement_unit = new RevIMU(HardwareMapContainer.getMap());
        internal_measurement_unit.init();
        //remember to account for size of robot when inputting coordinates.
        //Can we standardise placing these coordinates behind of the junction
        //test data:
        // {{0.5,0},{0.5,0.5}}


        double[][] junction_Coordinates = {{304.8, 2998}, {1828.8, 2998}, {1828.8, 1878.8}};
        int num_junctions = 3;

        //Is it possible to use the terminals as a way to check coordinates?

        //remember to account for size of robot when inputting coordinates.
        //Can we standardise placing these coordinates at the center of the terminal
        //test data:
        // {{0,0},{2,2}}
        double[][] terminal = {{50, 1878.8}, {50, 50}};
        //remember to account for size of robot when inputting coordinates. (assuming robot is 50mm in
        //length)
        //test data:
        //{0,2}
        double[][] cone_coordinates = {{1474, 3607.6}, {1474, 25}};

        //used later for finding closest cone stack
        double[] closest_coordinate = {cone_coordinates[0][0], cone_coordinates[0][1]};
        double distance = 0.0;
        double smallest_distance = 0.0;
        //assuming team 2's robot is placed on the right
        double[] current = {0, 2743.2};
        int junction_number = 0;
        double[] next = {junction_Coordinates[junction_number][0], junction_Coordinates[junction_number][1]};
        waitForStart();
        // Code to run once PLAY is pressed
        boolean has_cone = true;
        runtime.reset();
        double idealheading = 0.00;
        double time = 0.0;
        double elapsed = 0.0;
        double start_elapsed = 0.0;
        double speed = 0;
        //speed of motor1
        double speed1 = 0;
        //speed of motor2
        double speed2 = 0;
        double travelled = 0.0;
        double startheading = 0;
        motor1.resetEncoder();
        motor2.resetEncoder();
        boolean is_turning = false;
        // Run until the driver presses STOP
        double start_time = 0.0;
        int i = 0;
        while (opModeIsActive()) {
            // Code to run in a loop
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //Telling the robot to automatically break from driving loop when 10s left to return to base
            while (time < 20) {
                telemetry.addData("entered first loop", runtime.toString());
                //remember that the speed is in rpm, and the gear ratio of motor to the wheel is 15:20
                //3:4 gear ratio and the wheel is 85mm in diameter, which should equate to 1.4167 mm per turn
                speed1 = motor1.getCorrectedVelocity();
                speed2 = motor2.getCorrectedVelocity();
                //checking if the robot is turning except that motor 1 goes in reverse so the speed is
                //counted using speed 2.
                if (!is_turning) {
                    // I just want speed so i will get the absolute value
                    speed = Math.abs(speed2)*0.75*1.4167;
                } else {
                    speed = 0;
                }
                //finding total time that has elapsed
                time = runtime.seconds();
                //finding time elapsed for previous tick
                elapsed = time - start_elapsed;
                //resetting the new tick time from this moment
                start_elapsed = time;
                telemetry.addData("time per tick", elapsed);
                //finding distance travelled in last tick
                travelled = speed * elapsed;
                //updating current x coordinate
                current[0] = current[0] + travelled * Math.cos(idealheading);
                //updating current y coordinate
                current[1] = current[1] + travelled * Math.sin(idealheading);
                //Distance from target in terms of x distance and y distance
                if (!has_cone) {
                    telemetry.addData("Entered cone pick up", "x_coordinate" +current[0]);
                    telemetry.addData("Entered cone pick up", "y_coordinate" +current[1]);
                    //Run code to return to nearest loading station
                    //finding distance from each robot to cones using pythagoras
                    //assuming that the first coordinate is the closest
                    smallest_distance = Math.pow(current[0] - cone_coordinates[0][0], 2);
                    smallest_distance = smallest_distance + Math.pow(current[1] - cone_coordinates[0][1], 2);
                    //running through all the cone coordinates to find the coordinate with the
                    //lowest distance
                    for (int index = 1; index < cone_coordinates.length; index++) {
                        //finding distance using pythagoras
                        distance = Math.pow(current[0] - cone_coordinates[index][0], 2);
                        distance = distance + Math.pow(current[1] - cone_coordinates[index][1], 2);
                        if (distance < smallest_distance) {
                            //updating closest cones if the distance is smaller than the current "
                            //closest coordinate
                            smallest_distance = distance;
                            closest_coordinate = cone_coordinates[index];
                            i= index;
                        }
                    }
                    //getting the robot to drive towards the cones
                    //allowance of 1cm from pole, though subject to change as necessary
                    while (Math.abs(current[0] - closest_coordinate[0]) > 10 && Math.abs(current[1] - closest_coordinate[1]) > 10) {
                        //insert code for proximity sensor
                        speed1 = motor1.getCorrectedVelocity();
                        speed2 = motor2.getCorrectedVelocity();
                        //checking if the robot is turning except that motor 1 goes in reverse so the speed is
                        //counted using speed 2.
                        if (!is_turning) {
                            // I just want speed so i will get the absolute value
                            speed = Math.abs(speed2)*0.75*1.4167;
                        } else {
                            speed = 0;
                        }
                        time = runtime.seconds();
                        //finding time elapsed for previous tick
                        elapsed = time - start_elapsed;
                        //resetting the new tick time from this moment
                        start_elapsed = time;
                        telemetry.addData("time", elapsed);
                        travelled = speed * elapsed;
                        //updating current x coordinate
                        current[0] = current[0] + travelled * Math.cos(idealheading);
                        //updating current y coordinate
                        current[1] = current[1] + travelled * Math.sin(idealheading);
                        //finding heading required to drive towards coordinate
                        if (current[0] - closest_coordinate[0] < 0) {
                            //increase x coordinate
                            idealheading = 0;
                        } else if (current[0] - closest_coordinate[0] > 0) {
                            //decrease x coordinate
                            idealheading = 180;
                        } else if (current[1] - closest_coordinate[1] < 0) {
                            //increase y coordinate
                            idealheading = 90;
                        } else if (current[1] - closest_coordinate[1] > 0) {
                            //increase y coordinate
                            idealheading = 270;
                        }
                        startheading = internal_measurement_unit.getHeading();
                        //allowing an error of 0.5 degree as long as it is constantly updated.
                        if (startheading < idealheading + 0.5 && startheading > idealheading - 0.5) {
                            is_turning = false;
                            telemetry.addData("Driving forwards", motor2.getCorrectedVelocity());
                            //drive at reduced speeds if within 20cm of terminal
                            if(Math.abs(current[0]-closest_coordinate[0])<200 && Math.abs(current[1]-closest_coordinate[1])<200) {
                                motor1.set(-0.3);
                                motor2.set(0.3);
                            } else{
                                motor1.set(-0.8);
                                motor2.set(0.8);
                            }
                        } else if ((idealheading <startheading && startheading - idealheading<180) || (startheading <idealheading && 360-idealheading+startheading<180)) {
                            is_turning=true;
                            //Cause the motor to turn left
                            //10 degree error allowance for reducing speed is stopgap, need to input degree
                            //change generated by 1 tick of maximum turning
                            if (startheading - idealheading < 30 && startheading - idealheading > -30) {
                                //0.1 is a stopgap
                                motor1.set(0.05);
                                motor2.set(0.05);
                            } else {
                                motor1.set(0.3);
                                motor2.set(0.3);
                            }
                        } else {
                            is_turning=true;
                            //Cause the motor to turn right
                            //10 degree error allowance for reducing speed is stopgap, need to input degree
                            //change generated by 1 tick of maximum turning
                            if (startheading - idealheading < 30 && startheading - idealheading > -30) {
                                //0.1 is a stopgap
                                telemetry.addData("reduced turning", motor2.getCorrectedVelocity());
                                motor1.set(-0.05);
                                motor2.set(-0.05);
                            } else {
                                motor1.set(-0.3);
                                motor2.set(-0.3);

                            }
                        }
                        telemetry.update();
                    }
                    //getting the robot to stop
                    motor1.set(0);
                    motor2.set(0);
                    //code for picking up cone
                    if (i == 0){
                        //the cones on the right of the field
                        idealheading=45;
                    }else{
                        //the cones on the left of the field
                        idealheading=315;
                    }
                    //turning to face the junction
                    while (startheading < idealheading + 0.05 && startheading > idealheading - 0.05) {
                        is_turning = true;
                        if ((idealheading < startheading && startheading - idealheading < 180) || (startheading < idealheading && 360 - idealheading + startheading < 180)) {
                            //Cause the motor to turn left
                            //10 degree error allowance for reducing speed is stopgap
                            if (startheading - idealheading < 10 && startheading - idealheading > -10) {
                                //0.1 is a stopgap
                                motor1.set(0.05);
                                motor2.set(0.05);
                            } else {
                                motor1.set(0.3);
                                motor2.set(0.3);
                            }

                        } else {
                            //Cause the motor to turn right
                            //10 degree error allowance for reducing speed is stopgap
                            if (startheading - idealheading < 0 && startheading - idealheading > -10) {
                                motor1.set(-0.05);
                                motor2.set(-0.05);
                            } else {
                                motor1.set(-0.3);
                                motor2.set(-0.3);
                            }
                        }
                    }
                    start_time=runtime.seconds();
                    //get the robot to close the servo
                    left_intake.close();
                    right_intake.close();
                    //get the robot to lift the cone for 1s
                    while (runtime.seconds()-start_time<1) {
                        left_arm.set(-1);
                        right_arm.set(-1);
                    }
                    has_cone = true;

                    //should we enter code for the robot to drive to the nearest picture in order
                    //to reconfirm coordinates
                } else {
                    telemetry.addData("Entered cone deliver", "x_coordinate" +current[0]);
                    telemetry.addData("Entered cone deliver", "y_coordinate" +current[1]);
                    //allowance of 1cm from pole, though subject to change as necessary
                    if (Math.abs(current[0] - next[0]) > 10 && Math.abs(current[1] - next[1]) > 10) {
                        //Calculating heading necessary to go in correct direction
                        if (current[0] - next[0] < 0) {
                            //increase x coordinate
                            idealheading = 0;
                        } else if (current[0] - next[0] > 0) {
                            //decrease x coordinate
                            idealheading = 180;
                        } else if (current[1] - next[1] < 0) {
                            //increase y coordinate
                            idealheading = 90;
                        } else if (current[1] - next[1] > 0) {
                            //increase y coordinate
                            idealheading = 270;
                        }
                        startheading = internal_measurement_unit.getHeading();
                        //allowing an error of 0.5 degree as long as it is constantly updated.
                        if (startheading < idealheading + 0.5 && startheading > idealheading - 0.5) {
                            is_turning=false;
                            telemetry.addData("driving", motor2.getCorrectedVelocity());
                            //reduce speed if within 20cm of junction
                            if(Math.abs(current[0]-next[0])<200 && Math.abs(current[1]-next[1])<200) {
                                motor1.set(-0.3);
                                motor2.set(0.3);
                            } else{
                                motor1.set(-0.8);
                                motor2.set(0.8);
                            }
                        } else if ((idealheading <startheading && startheading - idealheading<180) || (startheading <idealheading && 360-idealheading+startheading<180)) {
                            is_turning=true;
                            //Cause the motor to turn left
                            //10 degree error allowance for reducing speed is stopgap
                            if (startheading - idealheading < 10 && startheading - idealheading > -10) {
                                //0.3 is a stopgap
                                motor1.set(0.05);
                                motor2.set(0.05);
                            } else {
                                motor1.set(0.3);
                                motor2.set(0.3);
                            }

                        } else {
                            is_turning=true;
                            //Cause the motor to turn right
                            //10 degree error allowance for reducing speed is stopgap
                            if (startheading - idealheading < 10 && startheading - idealheading > -10) {
                                //0.3 is a stopgap
                                motor1.set(-0.05);
                                motor2.set(-0.05);
                            } else {
                                motor1.set(-0.3);
                                motor2.set(-0.3);
                            }
                        }
                    } else {
                        //getting the robot to stop
                        motor1.set(0.0);
                        motor2.set(0.0);
                        startheading=internal_measurement_unit.getHeading();
                        //code to put down cone on Junction
                        idealheading=180;
                        while (startheading < idealheading + 0.05 && startheading > idealheading - 0.05) {
                            is_turning = true;
                            if ((idealheading < startheading && startheading - idealheading < 180) || (startheading < idealheading && 360 - idealheading + startheading < 180)) {
                                //Cause the motor to turn left
                                //10 degree error allowance for reducing speed is stopgap
                                if (startheading - idealheading < 10 && startheading - idealheading > -10) {
                                    //0.1 is a stopgap
                                    motor1.set(0.05);
                                    motor2.set(0.05);
                                } else {
                                    motor1.set(0.3);
                                    motor2.set(0.3);
                                }

                            } else {
                                //Cause the motor to turn right
                                //10 degree error allowance for reducing speed is stopgap
                                if (startheading - idealheading < 0 && startheading - idealheading > -10) {
                                    motor1.set(-0.05);
                                    motor2.set(-0.05);
                                } else {
                                    motor1.set(-0.3);
                                    motor2.set(-0.3);
                                }
                            }
                        }
                        start_time=runtime.seconds();
                        //get the robot to drop the cone for 1s
                        while (runtime.seconds()-start_time<1) {
                            left_arm.set(1);
                            right_arm.set(1);
                        }
                        //opening the intake
                        left_intake.setPosition(1.0);
                        right_intake.setPosition(1.0);
                        has_cone = false;
                        //incrementing junction number so that we do not put a cone on the same junction
                        junction_number++;
                        if (junction_number < num_junctions) {
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
                telemetry.addData("returning to terminal", "x_coordinate" +current[0]);
                telemetry.addData("returning to terminal", "y_coordinate"+ current[1]);
                //write code for returning to terminals
                speed1 = motor1.getCorrectedVelocity();
                speed2 = motor2.getCorrectedVelocity();
                //checking if the robot is turning except that motor 1 goes in reverse so the speed is
                //counted using speed 2.
                if (is_turning==false) {
                    // I just want speed so i will get the absolute value
                    speed = Math.abs(speed2)*1.4167;
                } else {
                    //if the robot is turning, then it technically has not changed position
                    speed = 0;
                }
                //finding total time that has elapsed
                time = runtime.seconds();
                //finding time elapsed for previous tick
                elapsed = time - start_elapsed;
                telemetry.addData("time taken", elapsed);
                //resetting the new tick time from this moment
                start_elapsed = time;
                travelled = speed * elapsed;
                //updating current x coordinate
                current[0] = current[0] + travelled * Math.cos(idealheading);
                //updating current y coordinate
                current[1] = current[1] + travelled * Math.sin(idealheading);
                //redefining closest terminal as the first terminal in list
                closest_coordinate[0] = terminal[0][0];
                closest_coordinate[1] = terminal[0][1];
                //finding distance from each terminal to find most efficient path to terminal
                //using pythagoras
                smallest_distance = Math.pow(current[0] - terminal[0][0], 2);
                smallest_distance = smallest_distance + Math.pow(current[1] - terminal[0][1], 2);
                //running through all the terminal coordinates to find the coordinate with the
                //lowest distance
                for (int index = 1; index < terminal.length; index++) {
                    //finding distance using pythagoras
                    distance = Math.pow(current[0] - terminal[index][0], 2);
                    distance = distance + Math.pow(current[1] - terminal[index][1], 2);
                    if (distance < smallest_distance) {
                        //updating closest cones if the distance is smaller than the current "
                        //closest coordinate
                        smallest_distance = distance;
                        closest_coordinate = cone_coordinates[index];
                    }
                }
                //allowance of 1cm from terminal, but is subject to change.
                while (Math.abs(current[0] - closest_coordinate[0]) < 10 && Math.abs(current[1] - closest_coordinate[1]) < 10) {
                    startheading = internal_measurement_unit.getHeading();
                    if (current[0] - closest_coordinate[0] < 0) {
                        //increase x coordinate
                        idealheading = 0;
                    } else if (current[0] - closest_coordinate[0] > 0) {
                        //decrease x coordinate
                        idealheading = 180;
                    } else if (current[1] - closest_coordinate[1] < 0) {
                        //increase y coordinate
                        idealheading = 90;
                    } else if (current[1] - closest_coordinate[1] > 0) {
                        //increase y coordinate
                        idealheading = 270;
                    }
                    //allowing an error of 0.5 degree as long as it is constantly updated.
                    if (startheading < idealheading + 0.05 && startheading > idealheading - 0.05) {
                        is_turning=false;
                        //drive at reduced speeds if within 20cm of terminal
                        if(Math.abs(current[0]-closest_coordinate[0])<200 && Math.abs(current[1]-closest_coordinate[1])<200) {
                            motor1.set(-0.3);
                            motor2.set(0.3);
                        } else{
                            motor1.set(-0.8);
                            motor2.set(0.8);
                        }
                    } else if ((idealheading <startheading && startheading - idealheading<180) || (startheading <idealheading && 360-idealheading+startheading<180)) {
                        is_turning=true;
                        //Cause the motor to turn left
                        //10 degree error allowance for reducing speed is stopgap
                        if (startheading - idealheading < 10 && startheading - idealheading > -10) {
                            //0.1 is a stopgap
                            motor1.set(0.05);
                            motor2.set(0.05);
                        } else {
                            motor1.set(0.3);
                            motor2.set(0.3);
                        }

                    } else {
                        is_turning=true;
                        //Cause the motor to turn right
                        //10 degree error allowance for reducing speed is stopgap
                        if (startheading - idealheading < 0 && startheading - idealheading > -10) {
                            motor1.set(-0.05);
                            motor2.set(-0.05);
                        } else {
                            motor1.set(-0.3);
                            motor2.set(-0.3);
                        }
                    }
                    telemetry.update();
                }
                //getting the robot to stop
                motor1.set(0.0);
                motor2.set(0.0);
            }
        }
    }

    private void driveToSignalZone(int zone, Motor leftMotor, Motor rightMotor) {
        switch (zone) {
            case 1: {
                driveStraight(5, leftMotor, rightMotor);
                break;
            }

            case 2: {
                driveStraight(5, leftMotor, rightMotor);
                rightAngleTurn(leftMotor, rightMotor, true);
                driveStraight(5, leftMotor, rightMotor);
                break;
            }

            case 3: {
                driveStraight(5, leftMotor, rightMotor);
                rightAngleTurn(leftMotor, rightMotor, true);
                driveStraight(10, leftMotor, rightMotor);
                break;
            }

            /*
            default: {

            }

             */
        }
    }

    private void driveStraight(double time, Motor leftMotor, Motor rightMotor) {
        
    }

    private void rightAngleTurn(Motor leftMotor, Motor rightMotor, boolean isClockwise) {
        double[] motorValues = new double[2];
        motorValues[0] = isClockwise ? 1D : -1D;
        motorValues[1] = -motorValues[0];

        leftMotor.set(motorValues[0]);
        leftMotor.set(motorValues[1]);
    }
}