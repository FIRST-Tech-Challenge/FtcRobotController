package org.firstinspires.ftc.teamcode.autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="Deliver Wobble", group ="Robot15173")
@Disabled

public class DeliverWobble extends AutoBase{
    @Override
    public void runOpMode() throws InterruptedException {
        runAutoMode();
    }
    @Override
    protected void initRobot(){
        super.initRobot();
    }

    @Override
    protected void act() {
        super.act();
        try {
            robot.closeWobbleClaw();
            sleep(500);

        }
        catch (Exception ex){
            telemetry.addData("Error", ex.getMessage());
            telemetry.update();
        }
    }
}
