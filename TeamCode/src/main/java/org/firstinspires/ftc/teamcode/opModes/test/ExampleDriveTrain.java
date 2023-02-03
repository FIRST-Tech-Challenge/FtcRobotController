package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Gyroscope;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;
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
    public void Autonomous(){
        //Code for autonomous running
        //Coordinate in meters from start point in terms of x and y
        double[][] pole_Coordinates={};
        double[][] dock_Coordinates={};
        double[] current={0,0};
        int i=0
        while(time<20){
            //Distance from target in terms of x distance and y distance
            double[] next = pole_Coordinates[i];
            double[] distance={} ;
            distance[0]= current[0]-next[0];
            distance[1]= current[1]-next[1];
            double idealheading= Math.atan2(distance[1],distance[0]);
            double startheading = internal_measurement_unit.getAbsoluteHeading();
            if(startheading==idealheading){
                //Drive at full speed towards target
                motor1.set(1)
            } else if ((startheading-idealheading>0)|| (startheading-idealheading<-180)) {
                //Cause the motor to turn left
                motor1.set(-1);
                motor2.set(1);

            } else{
                //Cause the motor to turn right
                motor1.set(1);
                motor2.set(-1);
            }
            //find a way to tell if you have arrived
        }





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
