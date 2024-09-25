

import com.qualcomm.robotcore.eventloop.opmode.*;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Auto")
public class Auto extends LinearOpMode {
    private CoyotesRobot robot;

    private StartingPosition startingPosition;

    /**
     * Automatically runs after pressing the "Init" button on the Control Hub
     */
    @Override
    public void runOpMode() {
        robot = new CoyotesRobot(this);

        startingPosition = ;

        // Wait until the player press the start button
        waitForStart();
    }
}