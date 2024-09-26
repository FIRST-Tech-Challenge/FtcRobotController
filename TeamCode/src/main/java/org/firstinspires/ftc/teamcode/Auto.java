

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    private CoyotesRobot robot;

    // Red or blue team
    private TeamColor teamColor;
    // Far or near
    private TeamSide teamSide;

    /**
     * Automatically runs after pressing the "Init" button on the Control Hub
     */
    @Override
    public void runOpMode() {
        robot = new CoyotesRobot(this);

        // Wait until the player press the start button
        waitForStart();
    }
}