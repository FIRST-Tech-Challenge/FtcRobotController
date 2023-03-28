package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousLinearModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;

//This code literally just gets to robot to drive left by 3 feet so that it ends up in an alliance terminal
//or substation just to score 2 autonomous points
@Autonomous(name="<Emergency_autonomous_red>", group="<OpMode group name>")
public class Contingency_plan extends AutonomousLinearModeBase {


    @Override
    public void run() {
        // Declare class members here
        ElapsedTime runtime = new ElapsedTime();
        RevIMU internal_measurement_unit;
        //left wheel
        Motor motor1 = HardwareMapContainer.motor0;
        //right wheel
        Motor motor2 = HardwareMapContainer.motor1;
        internal_measurement_unit = new RevIMU(HardwareMapContainer.getMap());
        internal_measurement_unit.init();


        double[] current = {0, 0};
        waitForStart();
        // Code to run once PLAY is pressed
        runtime.reset();
        double startheading=0;
        double idealheading=0;
        double elapsed=0;
        double start_elapsed=0;
        double travelled=0;
        double speed=0;
        double speed1=0;
        double speed2=0;
        boolean is_turning=false;
        double[] closest_coordinate={0,-864.4};
        while (opModeIsActive()) {
            // Code to run in a loop
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            //Telling the robot to automatically break from driving loop when 10s left to return to base
            while (time < 20) {
                telemetry.addData("returning to terminal", "x_coordinate" + current[0]);
                telemetry.addData("returning to terminal", "y_coordinate" + current[1]);
                //write code for returning to terminals
                speed1 = motor1.getCorrectedVelocity();
                speed2 = motor2.getCorrectedVelocity();
                //checking if the robot is turning, but since motor1 goes in reverse, the sum should be around 0
                if (is_turning == false) {
                    speed = Math.abs(speed2) * 1.4167;
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

                while (Math.abs(current[0] - closest_coordinate[0]) < 10 && Math.abs(current[1] - closest_coordinate[1]) < 10) {
                    startheading = internal_measurement_unit.getHeading();
                    //allowing an error of 0.5 degree as long as it is constantly updated.
                    if (startheading < idealheading + 0.05 && startheading > idealheading - 0.05) {
                        is_turning = false;
                        //drive at reduced speeds if within 20cm of terminal
                        if (Math.abs(current[0] - closest_coordinate[0]) < 200 && Math.abs(current[1] - closest_coordinate[1]) < 200) {
                            motor1.set(-0.3);
                            motor2.set(0.3);
                        } else {
                            motor1.set(-0.8);
                            motor2.set(0.8);
                        }
                    } else if ((idealheading < startheading && startheading - idealheading < 180) || (startheading < idealheading && 360 - idealheading + startheading < 180)) {
                        is_turning = true;
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
                        is_turning = true;
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
}
