/**
 * shooter servo testing
 *
 * @author  Nikhil
 * @version 1.0
 * @since   2020-October-26
 *
 */

package org.firstinspires.ftc.teamcode.Qualifier_1.Autonomous.Tests;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Qualifier_1.Components.Accesories.Shooter;

@Autonomous(name= "ShooterServoTest")
//@Disabled
public class ShooterServoTest extends LinearOpMode{
    private Shooter shooter=null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Ready to go");
        telemetry.update();
        telemetry.addData("Status", "InitComplete, Ready to Start");
        telemetry.update();


        waitForStart();
        shooter.moveServoPosition(1.0);
    }
}
