package org.firstinspires.ftc.teamcode.opModes.test;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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


@TeleOp(name="ExampleDriveTrain", group="Demo")
public class SPAMTACTICSExampleDriveTrain extends TeleOpModeBase {
    // Declare class members here
    //left wheel
    Motor motor1 = HardwareMapContainer.motor0;
    //right wheel
    Motor motor2 = HardwareMapContainer.motor1;
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
            motor1.set(-rightX);
            motor2.set(rightX);
        } else if ((startheading>nextheading&& startheading-nextheading<180) || (startheading<nextheading && 360-nextheading+startheading<180)) {
            //Cause the motor to turn left
            if (Math.abs(startheading - nextheading) < 10 ) {
                //0.1 is a stopgap
                motor1.set(-0.05);
                motor2.set(0.05);
            } else {
                motor1.set(-0.3);
                motor2.set(0.3);
            }

        } else{
            //Cause the motor to turn right
            if (Math.abs(startheading - nextheading)<10) {
                motor1.set(0.05);
                motor2.set(-0.05);
            } else {
                motor1.set(0.3);
                motor2.set(-0.3);
            }
        }
    }
}
