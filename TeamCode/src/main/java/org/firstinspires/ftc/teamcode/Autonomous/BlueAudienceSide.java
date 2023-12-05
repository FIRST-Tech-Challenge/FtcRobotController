package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.RobotClass;

@Autonomous()

/* Rowan's Notes:
    * Change the name for the class to BlueAudience to have consistency
 */
public class BlueAudienceSide extends LinearOpMode {

    RobotClass teamBot = new RobotClass(this);

    @Override
    public void runOpMode() throws InterruptedException {
        teamBot.init(hardwareMap);

        waitForStart();

        /* Rowan's Notes:
            * Why is there a loop for constantly running the program?
            * Move line under the corresponding comment. This allows people to see what line does what
         */
        while (opModeIsActive()){
            teamBot.moveWithoutEncoders(0.6, 0.6, 2500);
            teamBot.moveWithoutEncoders(0.6, 0.6, 1500);
            teamBot.gyroTurning(-90);
            teamBot.moveWithoutEncoders(0.6, 0.6, 6000);
            teamBot.strafing(RobotClass.Direction.LEFT, 0.5, 1000);
        }

        //Moving to spike mark grid square

        //Move forwards one more square

        //turning 90 degrees

        //move forwards to the wall

    }
}
