package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.ActualRobotDanceOpMode.STATE_OF_ROBOT.*;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.concurrent.TimeUnit;

@Autonomous(name = "REAL_DANCE_TIME")
public class ActualRobotDanceOpMode extends RobotOpMode {

    public STATE_OF_ROBOT currentState;

    enum STATE_OF_ROBOT {
        START,
        DANCE,
        FINISH
    }

    @Override
    public void robotloop() {
        switch (currentState) {
            case START:
                currentState = STATE_OF_ROBOT.DANCE;
                elapsedTime.reset();
                break;
            case FINISH:
                // do stuff at the end
                resetDriveMotors();
                terminateOpModeNow();
                break;
            case DANCE:
                // dance
                long time = elapsedTime.startTimeNanoseconds()+TimeUnit.SECONDS.convert(8, TimeUnit.NANOSECONDS);
                double secondsElapsed = elapsedTime.seconds();

                double speed = 3;
                double axial = Math.round(Math.sin((secondsElapsed*Math.PI)/speed));
                double lateral = Math.round(Math.cos((secondsElapsed*Math.PI)/speed));
                double yaw = 0;
                boolean finished = moveRobot(axial, lateral, yaw, time);

                if(finished) {
                    currentState = FINISH;
                }

                break;
            default:
                currentState = START;
                break;
        }
    }
}
