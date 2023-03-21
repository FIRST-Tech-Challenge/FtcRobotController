package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousLinearModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;

//test log: team 1 has been tested, just remember tha motor1/left motor goes in reverse
@Autonomous(name="<Turning>", group="<OpMode group name>")
public class Turning_Autonomous extends AutonomousLinearModeBase {

//test log: turns out that the robot measures distance in something like mm
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
        waitForStart();
        // Code to run once PLAY is pressed
        boolean has_cone = true;
        runtime.reset();
        double idealheading = 0.00;
        double speed = 0;
        //speed of motor1
        double speed1 = 0;
        //speed of motor2
        double speed2 = 0;
        double startheading = 0;
        motor1.resetEncoder();
        motor2.resetEncoder();
        double elapsed=0;
        double travelled=0;
        double start_elapsed=0;
        double [] current={0,0};
        // Run until the driver presses STOP
        boolean is_turning=false;
        while (opModeIsActive()) {
            //testing the robot driving forwards
            telemetry.addData("total", runtime.toString());
            speed1 = motor1.getCorrectedVelocity();
            speed2 = motor2.getCorrectedVelocity();
            //checking if the robot is turning except that motor 1 goes in reverse so the sum should be around 0 if it is going straight
            if (is_turning==false) {
                speed = speed2;
            } else {
                speed = 0;
            }
            telemetry.addData("forward speed", speed);
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
            telemetry.addData(    "xcoordinate",current[0]);
            //99.5 units = 145mm
            //150 units = 190mm
            //204 units = 291mm
            telemetry.addData("ycoordinate", current[1]);
            //Distance from target in terms of x distance and y distance
            motor1.set(-0.1);
            motor2.set(0.1);
            //when testing turning speeds, testing right turn
            //motor1.set(0.3)
            //motor2.set(0.3)
            telemetry.update();
        }


    }
}