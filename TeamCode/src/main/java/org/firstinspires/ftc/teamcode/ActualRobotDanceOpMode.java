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
        if(currentState == null) {
            currentState = START;
        }
        switch (currentState) {
            case START:
                currentState = STATE_OF_ROBOT.DANCE;
                telemetry.speak("it is dance time! woo hoo! yay its dance time!", "ger", "DE");
                elapsedTime.reset();
                break;
            case FINISH:
                // do stuff at the end
                resetDriveMotors();
                requestOpModeStop();
                break;
            case DANCE:
                // dance
                long time = elapsedTime.startTimeNanoseconds()+TimeUnit.SECONDS.convert(8, TimeUnit.NANOSECONDS);
                double secondsElapsed = elapsedTime.seconds();

                double speed = 3;
                double axial = Math.round(Math.sin((secondsElapsed*Math.PI)/speed));
                double lateral = Math.round(Math.cos((secondsElapsed*Math.PI)/speed));
                double yaw = Math.sin((-secondsElapsed*Math.PI)/speed*1.5d)/2;
                boolean finished = moveRobot(axial, lateral, yaw, time);

                if(finished) {
                    currentState = FINISH;
                    telemetry.speak("womp womp");
                }
                break;
        }
        telemetry.addData("CURRENT STATE: ", currentState);
    }
}
