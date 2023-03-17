package org.firstinspires.ftc.teamcode.opModes.test;

import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousLinearModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.modeBases.AutonomousModeBase;
import org.firstinspires.ftc.teamcode.libs.brightonCollege.util.HardwareMapContainer;


@Autonomous(name="<Autonomous>", group="<OpMode group name>")
public class Turning_Autonomous extends AutonomousLinearModeBase {


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
        // Run until the driver presses STOP
        while (opModeIsActive()) {
            // Code to run in a loop
            //turning right version 1
            telemetry.addData("Time", runtime.toString());
            motor1.set(1.0);
            telemetry.addData("leftmotor", motor1.getCorrectedVelocity());
            motor2.set(-1.0);
            telemetry.addData("rightmotor", motor2.getCorrectedVelocity());
            //turning right version 2
            //motor1.set(1.0);
            //telemetry.addData("leftmotor", motor1.getCorrectedVelocity());
            //motor2.set(0.0);
            //telemetry.addData("rightmotor", motor2.getCorrectedVelocity());
            telemetry.update();
        }

    }
}