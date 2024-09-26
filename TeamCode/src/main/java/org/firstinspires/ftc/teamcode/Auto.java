

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    private CoyotesRobot robot;

    private TeamColor teamColor;
    private TeamSide teamSide;

    /**
     * Automatically runs after pressing the "Init" button on the Control Hub
     */
    @Override
    public void runOpMode() {
        robot = new CoyotesRobot(this);

        startingPosition = StartingPosition.RED_NEAR;

        // Wait until the player press the start button
        waitForStart();
    }
}