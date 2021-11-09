package org.firstinspires.ftc.teamcode.Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Config.MainConfig;
import org.firstinspires.ftc.teamcode.Config.VisionConfig;
import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.Subsystems.Vision.DetectMarker.MarkerLocation;
import org.firstinspires.ftc.teamcode.Util.AllianceColor;


/**
 * Auto creates a robots and runs it in auto mode. This auto class is for when we are
 * on the blue alliance.
 *
 * <p>Auto currently just initializes the Robot as Auto.runOpMode() is empty.</p>
 *
 * @see com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
 */

//Tasks:
// Deliver duck from carousel (10)
//Deliver freight to hub (6)
// - deliver freight to corresponding level of custom element (20)
//Park in warehouse (10)
@Autonomous(name = "Auto Blue", group = "Auto")
public class AutoBlue extends Auto {
    /** Override of runOpMode()
     *
     * <p>Please do not swallow the InterruptedException, as it is used in cases
     * where the op mode needs to be terminated early.</p>
     *
     * @throws InterruptedException If the robot is terminated this is thrown.
     *
     * @see com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
     */
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = init(AllianceColor.BLUE);

        int placementLevel = getHubLevel();
    }
}
