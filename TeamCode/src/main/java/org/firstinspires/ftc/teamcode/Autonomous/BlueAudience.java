package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.RobotClass;

@Autonomous()

/* Rowan's Notes:
    * Change the name for the class to BlueAudience to have consistency
 */
public class BlueAudience extends LinearOpMode {

    RobotClass teamBot = new RobotClass(this);
    private Object posOfTag;


    @Override
    public void runOpMode() throws InterruptedException {
        teamBot.init(hardwareMap);
        posOfTag = teamBot.findTeamProp(502);

        waitForStart();

        //Moving to spike mark grid square
        teamBot.moveWithoutEncoders(0.6, 0.6, 2500);
        //Move forwards one more square
        teamBot.moveWithoutEncoders(0.6, 0.6, 1500);
        //turning 90 degrees
        teamBot.gyroTurning(-90);
        //move forwards to the wall
        teamBot.moveWithoutEncoders(0.6, 0.6, 6000);
        //Strafe for safety
        teamBot.strafing(RobotClass.Direction.LEFT, 0.5, 1000);
    }
}
