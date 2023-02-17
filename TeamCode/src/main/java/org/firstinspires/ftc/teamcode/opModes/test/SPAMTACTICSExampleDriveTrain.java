package org.firstinspires.ftc.teamcode.opModes.test;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Gyroscope;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;
// for autonomous mode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousModeBase;
// for sets
import java.util.*;
/**
 * Description: [Fill in]
 * Hardware:
 *  [motor0] Unused
 *  [motor1] Unused
 *  [motor2] Unused
 *  [motor3] Unused
 *  [servo0] Unused
 *  [servo1] Unused
 *  [servo2] Unused
 *  [servo3] Unused
 * Controls:
 *  [Button] Function
 */

@Autonomous(name="<OpMode name>", group="<OpMode group name>")
public class Autonomous_test extends AutonomousModeBase {
// Declare class members here
    private ElapsedTime runtime = new ElapsedTime();
    RevIMU internal_measurement_unit;
    @Override
    public void run() {
        //Creating a set to store coordinate of pole to prevent collisions
        Set<double> forbidden_coordinates = new HashSet<double>();
        //Code for autonomous running
        //Coordinate in meters from start point in terms of x and y
        internal_measurement_unit = new RevIMU(HardwareMapContainer.getMap());
        internal_measurement_unit.init();
        //remember to account for size of robot when inputting coordinates. Can we standardise placing these coordinates on the bottom left corner of the station
        double[][] pole_Coordinates={};
        //Is it possible to use the dock as a way to check coordinates?
        double[][] dock_Coordinates={};
        double[] current={0,0};
        int i=0;
        waitForStart();
        // Code to run once PLAY is pressed
        boolean has_cone = true;
        runtime.reset();
        //declaring idealheading
        double idealheading = 0.00;
        // Run until the driver presses STOP
        while (opModeIsActive()) {
            // Code to run in a loop
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.update();
            //converting runtime into a double
            double elapsed = double(runtime);
            //Telling the robot to automatically break from driving loop when 10s left to return to base
            while(elapsed<20) {
                if (elapsed>0){
                    //updating current x coordinate
                    current[0]= elapsed/Math.tan(idealheading);
                    //updating current y coordinate
                    current[1]= elapsed*Math.tan(idealheading)
                }
                //Distance from target in terms of x distance and y distance
                if (has_cone == false) {
                    //Run code to return to nearest loading station
                }
                double[] next = pole_Coordinates[i];
                double[] distance = {};
                distance.add(current[0] - next[0]);
                distance.add(current[1] - next[1]);
                idealheading = Math.atan2(distance[1], distance[0]);
                // Find a way to check if other poles are in the way to not collide
                double startheading = internal_measurement_unit.getAbsoluteHeading();
                if (startheading == idealheading) {
                    //Drive at full speed towards target
                    motor1.set(1);
                } else if ((startheading - idealheading > 0) || (startheading - idealheading < -180)) {
                    //Cause the motor to turn left
                    motor1.set(-1);
                    motor2.set(1);
                } else {
                    //Cause the motor to turn right
                    motor1.set(1);
                    motor2.set(-1);
                }
                //find a way to tell if you have arrived
            }
        }
    }
}
@TeleOp(name="Drivetrain Demo", group="Demo")
public class ExampleDriveTrain extends TeleOpModeBase {
    // Declare class members here
    //left wheel
    Motor motor1 = HardwareMapContainer.motor0;
    //right wheel
    Motor motor2 = HardwareMapContainer.motor1;
    //Motor motor3 = HardwareMapContainer.motor2;
    //Motor motor4 = HardwareMapContainer.motor3;
    RevIMU internal_measurement_unit;

    @Override
    public void setup() {
        // Code to run once after `INIT` is pressed
        internal_measurement_unit = new RevIMU(HardwareMapContainer.getMap());
        internal_measurement_unit.init();
    }

    @Override
    public void every_tick() {
        // Code to run in a loop after `PLAY` is pressed
        double startheading = internal_measurement_unit.getAbsoluteHeading();
        //The required heading
        double leftX= Inputs.gamepad1.getLeftX();
        double leftY = Inputs.gamepad1.getLeftY();
        double nextheading = Math.atan2(leftY,leftX);

        if(startheading==nextheading){
            //The necessary speed
            double rightX= Inputs.gamepad1.getRightX();
            motor1.set(rightX)
        } else if ((startheading-nextheading>0)|| (startheading-nextheading<-180)) {
            //Cause the motor to turn left
            motor1.set(-1);
            motor2.set(1);

        } else{
            //Cause the motor to turn right
            motor1.set(1);
            motor2.set(-1);
        }





    }
}
