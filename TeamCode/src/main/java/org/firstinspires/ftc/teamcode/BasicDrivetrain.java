package org.firstinspires.ftc.teamcode;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
public class BasicDrivetrain {
    private final DcMotor leftFront;
    private final DcMotor leftBack;
    private final DcMotor rightFront;
    private final DcMotor rightBack;

    public final int SMALL_NUMBER = 50;

    // these are instance variables to represent current motor positions.
    // in the ideal world they would all be 0 upon start and I would not need to initialize them in the constructor,
    // but in the real world there are tiny errors.

    // specifies current mode
    private State currentMode = State.DO_NOTHING;
    private HardwareMap h;
    private Telemetry t;

    public enum State {
        DO_NOTHING,
        MOVE
    }
    public BasicDrivetrain (LinearOpMode opMode) {
        this.h = opMode.hardwareMap;
        this.t = opMode.telemetry;

        this.leftBack = h.dcMotor.get("leftBack");
        this.leftFront = h.dcMotor.get("leftFront");
        this.rightBack = h.dcMotor.get("rightBack"); // if stupid runtime errors occur, just flip forward and back side motors.
        this.rightFront = h.dcMotor.get("rightFront"); // change the config to fix the names.

        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        this.leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        this.rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        t.addData("LeftBack Encoder", leftBack.getCurrentPosition());
        t.addData("LeftFront Encoder", leftFront.getTargetPosition());
        t.addData("RightBack Encoder", rightBack.getTargetPosition());
        t.addData("RightFront Encoder", rightFront.getTargetPosition());
    }

    public boolean positionsOnTarget() { // uses Target interface in LinearRobot class to create a stopping criterion
        return Math.abs(this.leftFront.getCurrentPosition() - this.leftFront.getTargetPosition()) < SMALL_NUMBER &&
                Math.abs(this.leftBack.getCurrentPosition()- this.leftBack.getTargetPosition()) < SMALL_NUMBER &&
                Math.abs(this.rightFront.getCurrentPosition() - this.rightFront.getTargetPosition()) < SMALL_NUMBER &&
                Math.abs(this.rightBack.getCurrentPosition() - this.rightBack.getTargetPosition()) < SMALL_NUMBER;
    }

    public void update() {
        t.addLine(this.currentMode.name());
        switch (currentMode) {
            /*
             * this case literally does nothing.
             * it used to reset the encoders but it doesn't.
             */
            case DO_NOTHING: // remove the reset-encoder thing because we are using the current position now
                break;

            case MOVE:
                this.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                this.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                break;
        }
    }

    public void start() {
        leftFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    // methods that will just change the state, separate for the sake of readability.
    public void doNothing() {
        currentMode = State.DO_NOTHING;
    }
}
