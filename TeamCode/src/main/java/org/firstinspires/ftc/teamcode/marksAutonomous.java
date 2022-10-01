package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import static org.firstinspires.ftc.teamcode.Variables.*;



@Autonomous(name ="MarkAuto", group = "A+")
public class marksAutonomous extends DriveMethods {


    @Override
    public void runOpMode() {

        initMotorsBlue();

        waitForStart();

        driveForDistance(2,1,Direction.FORWARD);


        while(opModeIsActive()) {

        }
    }

    public enum Direction {
        FORWARD,
        BACKWARD,
        ROTATE_LEFT,
        ROTATE_RIGHT,
        RIGHT,
        LEFT,

    }

    public void driveDirection(Direction direction, double power) {
        switch(direction) {
            case FORWARD:
                motorFL.setPower(power);
                motorBL.setPower(power);
                motorFR.setPower(power);
                motorBR.setPower(power);
                break;

            case BACKWARD:
                motorFL.setPower(-power);
                motorBL.setPower(-power);
                motorFR.setPower(-power);
                motorBR.setPower(-power);
                break;

            case RIGHT:
                motorFL.setPower(power);
                motorBL.setPower(-power);
                motorFR.setPower(-power);
                motorBR.setPower(power);
                break;

            case LEFT:
                motorFL.setPower(-power);
                motorBL.setPower(power);
                motorFR.setPower(power);
                motorBR.setPower(-power);
                break;

            case ROTATE_LEFT:
                motorFL.setPower(-power);
                motorBL.setPower(-power);
                motorFR.setPower(power);
                motorBR.setPower(power);
                break;

            case ROTATE_RIGHT:
                motorFL.setPower(power);
                motorBL.setPower(power);
                motorFR.setPower(-power);
                motorBR.setPower(-power);
                break;
        }
    }

    public void driveForTime (int seconds, double power,Direction direction) { //seconds: 10, power 1, direction: Forward

        driveDirection(direction, power);


        sleep(seconds * 1000);

        motorFL.setPower(0);
        motorBL.setPower(0);
        motorFR.setPower(0);
        motorBR.setPower(0);

    }

    public void driveForDistance (double distance, double power, Direction direction) {
        motorBL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorBR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorFR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int targetClicks;
        targetClicks = (int)(distance * rotationsPerMeter * clicksPerRotation);
        int currentClicks = 0;

        motorBL.setTargetPosition((targetClicks));
        motorFL.setTargetPosition((targetClicks));
        motorBR.setTargetPosition((targetClicks));
        motorFR.setTargetPosition((targetClicks));

        motorBL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorBR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorFR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



        while(currentClicks < targetClicks) {
            driveDirection(direction, power);
            currentClicks = (Math.abs(motorBL.getCurrentPosition()) +
            Math.abs(motorFL.getCurrentPosition()) +
            Math.abs(motorBR.getCurrentPosition()) +
            Math.abs(motorFR.getCurrentPosition()))/4;
            telemetry.addLine("Current Distance: " + currentClicks/clicksPerRotation/rotationsPerMeter);
            telemetry.update();
        }

        motorBL.setPower(0);
        motorFL.setPower(0);
        motorBR.setPower(0);
        motorFR.setPower(0);
    }
}
