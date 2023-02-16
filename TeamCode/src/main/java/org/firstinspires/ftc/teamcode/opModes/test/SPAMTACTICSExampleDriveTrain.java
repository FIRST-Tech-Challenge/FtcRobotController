package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.hardware.GyroEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Gyroscope;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.inputs.Inputs;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.TeleOpModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.TelemetryContainer;

/**
 * Description: [Fill in]
 * Hardware:
 *  [motor0] Left Wheel
 *  [motor1] Right Wheel
 *  [motor2] Unused
 *  [motor3] Unused
 *  [servo0] Unused
 *  [servo1] Unused
 *  [servo2] Unused
 *  [servo3] Unused
 * Controls:
 *  [Left Joystick] Set Heading
 *  [Right Joystick x] Set Speed towards heading
 */
@TeleOp(name="Drivetrain Demo [spamtactics]", group="Demo")
@Disabled
public class SPAMTACTICSExampleDriveTrain extends TeleOpModeBase { // TODO: Test fully with field if using; we had no field when this was created; sometimes throws an internal error from FTCLib
    // Declare class members here
    //left wheel
    Motor motor1 = HardwareMapContainer.motor0;
    //right wheel
    Motor motor2 = HardwareMapContainer.motor1;
    //Motor motor3 = HardwareMapContainer.motor2;
    //Motor motor4 = HardwareMapContainer.motor3;
    RevIMU internal_measurement_unit; // This works really well!

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

        //The necessary speed
        double rightX= Inputs.gamepad1.getRightX();

        TelemetryContainer.getTelemetry().addData("Speed", rightX);
        TelemetryContainer.getTelemetry().addData("NOW Heading", startheading);
        TelemetryContainer.getTelemetry().addData("AIM Heading", nextheading);

        if(startheading==nextheading){
            motor1.set(rightX);
            motor2.set(rightX);
        } else if ((startheading-nextheading>0)|| (startheading-nextheading<Math.PI)) { // Angles are in radians
            motor1.set(rightX);
            motor2.set(-rightX);

        } else{
            motor1.set(-rightX);
            motor2.set(rightX);
        }
    }
}
