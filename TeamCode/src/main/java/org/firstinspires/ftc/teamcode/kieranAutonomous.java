package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import static org.firstinspires.ftc.teamcode.Variables.*;

@Autonomous(name="KieAuto2", group = "X")

public class kieranAutonomous extends DriveMethods{



    public void runOpMode() {

        initMotorsBlue();


        waitForStart();

        driveForTime(10,1, Direction.FORWARD);

        driveForTime(5, 1, Direction.LEFT);

        driveForTime(5, 1, Direction.BACKWARD);

        driveForTime(5, 1, Direction.RIGHT);


        while(opModeIsActive()){

        }
    }

    public enum Direction{
        FORWARD,
        BACKWARD,
        ROTATE_LEFT,
        ROTATE_RIGHT,
        RIGHT,
        LEFT,q
    }

    public void driveDirection(Direction direction, double power) {
        switch(direction) {
            case FORWARD:
                motorFL.setPower(power);
                motorBL.setPower(power);
                motorBR.setPower(power);
                motorFR.setPower(power);

                break;

            case BACKWARD:
                motorFL.setPower(-power);
                motorBL.setPower(-power);
                motorBR.setPower(-power);
                motorFR.setPower(-power);

                break;

            case RIGHT:
                motorFL.setPower(power);
                motorBL.setPower(-power);
                motorBR.setPower(power);
                motorFR.setPower(-power);

                break;

            case LEFT:
                motorFL.setPower(-power);
                motorBL.setPower(power);
                motorBR.setPower(-power);
                motorFR.setPower(power);

                break;

            case ROTATE_RIGHT:
                motorFL.setPower(power);
                motorBL.setPower(power);
                motorBR.setPower(-power);
                motorFR.setPower(-power);

                break;

            case ROTATE_LEFT:
                motorFL.setPower(-power);
                motorBL.setPower(-power);
                motorBR.setPower(power);
                motorFR.setPower(power);

                break;
        }
    }

    public void driveForTime (int seconds, double power, Direction direction){

        driveDirection(direction, power);

        sleep(seconds*1000);

        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);

    }
}

