package org.firstinspires.ftc.teamcode.team10515.odometery;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

@Autonomous(name="EncoderBasedMecanumMethods", group = "Pushbot")

public abstract class ImportantMethods extends LinearOpMode {
    /* Declare OpMode members. */
    hwmap robot = new hwmap();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private double power;
    private int position;
    private double desiredAngle;
    private double current_heading;
    Orientation angles;
    public Orientation orientation;

    BNO055IMU imu;

    public double getPower() {
        return power;
    }

    public void setPower(double power) {
        this.power = power;
    }

    public int getPosition() {
        return position;
    }

    public void setPosition(int position) {
        this.position = position;
    }

    public double getDesiredAngle() { return desiredAngle; }

    public void setDesiredAngle(double desiredAngle) { this.desiredAngle = desiredAngle; }

    public void instant() {
        robot = new hwmap();
        runtime = new ElapsedTime();
        robot.init(hardwareMap);

    }

    public void forward() {
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setTargetPosition(robot.leftFrontDrive.getCurrentPosition() + position);
        robot.rightFrontDrive.setTargetPosition(robot.rightFrontDrive.getCurrentPosition() + position);
        robot.leftBackDrive.setTargetPosition(robot.leftBackDrive.getCurrentPosition() + position);
        robot.rightBackDrive.setTargetPosition(robot.rightBackDrive.getCurrentPosition() + position);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFrontDrive.setPower(power);
        robot.rightFrontDrive.setPower(power);
        robot.leftBackDrive.setPower(power);
        robot.rightBackDrive.setPower(power);

        while (robot.leftFrontDrive.isBusy() && robot.rightFrontDrive.isBusy() && robot.leftBackDrive.isBusy() && robot.rightBackDrive.isBusy()) {
            //should do nothing here


        }
        stopMoving();
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    public void backward() {
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setTargetPosition(robot.leftFrontDrive.getCurrentPosition() + -position);
        robot.rightBackDrive.setTargetPosition(robot.rightBackDrive.getCurrentPosition() + -position);
        robot.rightFrontDrive.setTargetPosition(robot.rightFrontDrive.getCurrentPosition() + -position);
        robot.leftBackDrive.setTargetPosition(robot.leftBackDrive.getCurrentPosition() + -position);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFrontDrive.setPower(-power);
        robot.rightFrontDrive.setPower(-power);
        robot.leftBackDrive.setPower(-power);
        robot.rightBackDrive.setPower(-power);


        while (robot.rightBackDrive.isBusy() && robot.rightFrontDrive.isBusy() && robot.leftFrontDrive.isBusy() && robot.leftBackDrive.isBusy()) {
            //should do nothing here
        }

        stopMoving();
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeLeft() {
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setTargetPosition(robot.leftFrontDrive.getCurrentPosition() + -position);
        robot.rightBackDrive.setTargetPosition(robot.rightBackDrive.getCurrentPosition() + -position);
        robot.rightFrontDrive.setTargetPosition(robot.rightFrontDrive.getCurrentPosition() + position);
        robot.leftBackDrive.setTargetPosition(robot.leftBackDrive.getCurrentPosition() + position);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFrontDrive.setPower(-power);
        robot.rightFrontDrive.setPower(power);
        robot.leftBackDrive.setPower(power);
        robot.rightBackDrive.setPower(-power);

        while (robot.rightBackDrive.isBusy() && robot.rightFrontDrive.isBusy() && robot.leftFrontDrive.isBusy() && robot.leftBackDrive.isBusy()) {
            //should do nothing here
        }

        stopMoving();
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void strafeRight() {
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setTargetPosition(robot.leftFrontDrive.getCurrentPosition() + position);
        robot.rightBackDrive.setTargetPosition(robot.rightBackDrive.getCurrentPosition() + position);
        robot.rightFrontDrive.setTargetPosition(robot.rightFrontDrive.getCurrentPosition() + -position);
        robot.leftBackDrive.setTargetPosition(robot.leftBackDrive.getCurrentPosition() + -position);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFrontDrive.setPower(power);
        robot.rightFrontDrive.setPower(-power);
        robot.leftBackDrive.setPower(-power);
        robot.rightBackDrive.setPower(power);

        while (robot.rightBackDrive.isBusy() && robot.rightFrontDrive.isBusy() && robot.leftFrontDrive.isBusy() && robot.leftBackDrive.isBusy()) {
            //should do nothing here
        }

        stopMoving();
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void diagonalRightForward() {
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setTargetPosition(robot.leftFrontDrive.getCurrentPosition() + position);
        robot.rightBackDrive.setTargetPosition(robot.rightBackDrive.getCurrentPosition() + position);
        robot.rightFrontDrive.setTargetPosition(robot.rightFrontDrive.getCurrentPosition() + -position);
        robot.leftBackDrive.setTargetPosition(robot.leftBackDrive.getCurrentPosition() + -position);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFrontDrive.setPower(power);
        robot.rightFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightBackDrive.setPower(power);

        while (robot.rightBackDrive.isBusy() && robot.rightFrontDrive.isBusy() && robot.leftFrontDrive.isBusy() && robot.leftBackDrive.isBusy()) {
            //should do nothing here
        }

        stopMoving();
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void diagonalRightBackward() {
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setTargetPosition(robot.leftFrontDrive.getCurrentPosition() + -position);
        robot.rightBackDrive.setTargetPosition(robot.rightBackDrive.getCurrentPosition() + -position);
        robot.rightFrontDrive.setTargetPosition(robot.rightFrontDrive.getCurrentPosition() + position);
        robot.leftBackDrive.setTargetPosition(robot.leftBackDrive.getCurrentPosition() + position);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFrontDrive.setPower(-power);
        robot.rightFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightBackDrive.setPower(-power);

        while (robot.rightBackDrive.isBusy() && robot.rightFrontDrive.isBusy() && robot.leftFrontDrive.isBusy() && robot.leftBackDrive.isBusy()) {
            //should do nothing here
        }

        stopMoving();
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void diagonalLeftForward() {
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setTargetPosition(robot.leftFrontDrive.getCurrentPosition() + -position);
        robot.rightBackDrive.setTargetPosition(robot.rightBackDrive.getCurrentPosition() + -position);
        robot.rightFrontDrive.setTargetPosition(robot.rightFrontDrive.getCurrentPosition() + position);
        robot.leftBackDrive.setTargetPosition(robot.leftBackDrive.getCurrentPosition() + position);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(power);
        robot.leftBackDrive.setPower(power);
        robot.rightBackDrive.setPower(0);

        while (robot.rightBackDrive.isBusy() && robot.rightFrontDrive.isBusy() && robot.leftFrontDrive.isBusy() && robot.leftBackDrive.isBusy()) {
            //should do nothing here
        }

        stopMoving();
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void diagonalLeftBackward() {
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setTargetPosition(robot.leftFrontDrive.getCurrentPosition() + position);
        robot.rightBackDrive.setTargetPosition(robot.rightBackDrive.getCurrentPosition() + position);
        robot.rightFrontDrive.setTargetPosition(robot.rightFrontDrive.getCurrentPosition() + -position);
        robot.leftBackDrive.setTargetPosition(robot.leftBackDrive.getCurrentPosition() + -position);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFrontDrive.setPower(0);
        robot.rightFrontDrive.setPower(-power);
        robot.leftBackDrive.setPower(-power);
        robot.rightBackDrive.setPower(0);

        while (robot.rightBackDrive.isBusy() && robot.rightFrontDrive.isBusy() && robot.leftFrontDrive.isBusy() && robot.leftBackDrive.isBusy()) {
            //should do nothing here
        }

        stopMoving();
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void clockwise() {
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setTargetPosition(robot.leftFrontDrive.getCurrentPosition() + position);
        robot.rightBackDrive.setTargetPosition(robot.rightBackDrive.getCurrentPosition() + -position);
        robot.rightFrontDrive.setTargetPosition(robot.rightFrontDrive.getCurrentPosition() + -position);
        robot.leftBackDrive.setTargetPosition(robot.leftBackDrive.getCurrentPosition() + position);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFrontDrive.setPower(power);
        robot.rightFrontDrive.setPower(-power);
        robot.leftBackDrive.setPower(power);
        robot.rightBackDrive.setPower(-power);

        while (robot.rightBackDrive.isBusy() && robot.rightFrontDrive.isBusy() && robot.leftFrontDrive.isBusy() && robot.leftBackDrive.isBusy()) {
            //should do nothing here
        }

        stopMoving();
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void counterclockwise() {
        robot.leftFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftFrontDrive.setTargetPosition(robot.leftFrontDrive.getCurrentPosition() + -position);
        robot.rightBackDrive.setTargetPosition(robot.rightBackDrive.getCurrentPosition() + position);
        robot.rightFrontDrive.setTargetPosition(robot.rightFrontDrive.getCurrentPosition() + position);
        robot.leftBackDrive.setTargetPosition(robot.leftBackDrive.getCurrentPosition() + -position);

        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFrontDrive.setPower(-power);
        robot.rightFrontDrive.setPower(power);
        robot.leftBackDrive.setPower(-power);
        robot.rightBackDrive.setPower(power);

        while (robot.rightBackDrive.isBusy() && robot.rightFrontDrive.isBusy() && robot.leftFrontDrive.isBusy() && robot.leftBackDrive.isBusy()) {
            //should do nothing here
        }

        stopMoving();
        robot.leftFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightFrontDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBackDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void turn_to_heading() {
        boolean go_right;
        double current_heading;
        double degrees_to_turn;
        double wheel_power;
        double prev_heading = 0;
        ElapsedTime timeout_timer = new ElapsedTime();

        current_heading = get_current_heading();
        degrees_to_turn = Math.abs(desiredAngle - current_heading);

        go_right = desiredAngle > current_heading;
        if (degrees_to_turn > 180) {
            go_right = !go_right;
            degrees_to_turn = 360 - degrees_to_turn;
        }

        timeout_timer.reset();
        prev_heading = current_heading;
        while (degrees_to_turn > .5 && opModeIsActive() && timeout_timer.seconds() < 2) {
            //wheel_power = (2 * Math.pow((degrees_to_turn + 13) / 30, 2) + 15) / 100;
            wheel_power = (0.85 * degrees_to_turn) / 100;
            if (go_right) {
                wheel_power = -wheel_power;
            }

            robot.rightFrontDrive.setPower(wheel_power);
            robot.rightBackDrive.setPower(wheel_power);
            robot.leftFrontDrive.setPower(-wheel_power);
            robot.leftFrontDrive.setPower(-wheel_power);

            current_heading = get_current_heading();
            degrees_to_turn = Math.abs(desiredAngle - current_heading);       // Calculate how far is remaining to turn

            go_right = desiredAngle > current_heading;

            if (degrees_to_turn > 180) {
                go_right = !go_right;
                degrees_to_turn = 360 - degrees_to_turn;
            }

            if (Math.abs(current_heading - prev_heading) > 1) {
                timeout_timer.reset();
                prev_heading = current_heading;
            }
        }
        stopMoving();
    }
    public double get_current_heading() {
        orientation = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        current_heading = orientation.firstAngle;

        if (current_heading < 0) {
            current_heading = -current_heading;
        } else {
            current_heading = 360 - current_heading;
        }
        current_heading = shift_heading(current_heading);
        return current_heading;
    }

    private double shift_heading(double heading) {
        double shiftvalue = 3;
        heading = heading + shiftvalue;

        if (heading >= 360) {
            heading = heading - 360;
        } else if (heading < 0) {
            heading = heading + 360;
        }
        return heading;
    }

    public void stopMoving() {
        robot.leftFrontDrive.setPower(0);
        robot.leftBackDrive.setPower(0);
        robot.rightFrontDrive.setPower(0);
        robot.rightBackDrive.setPower(0);
        setPosition(0);
        sleep(10);
    }
}