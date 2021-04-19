/**
 * shooter servo testing
 *
 * @author  Nikhil
 * @version 1.0
 * @since   2020-October-26
 *
 */

package org.firstinspires.ftc.teamcode.Autonomous.Tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.BasicChassis;
import org.firstinspires.ftc.teamcode.Robot;

@Autonomous(name= "ShooterServoTest ", group="Tests: ")
@Disabled
public class ShooterServoTest extends LinearOpMode{
    final Robot robot = new Robot(this, BasicChassis.ChassisType.ODOMETRY, false, false);

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Ready to go");
        telemetry.update();
        telemetry.addData("Status", "InitComplete, Ready to Start");
        telemetry.update();


        waitForStart();
        robot.moveServo(true);
    }
}
