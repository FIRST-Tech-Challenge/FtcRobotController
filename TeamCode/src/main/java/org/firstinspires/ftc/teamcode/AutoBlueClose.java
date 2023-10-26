package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
@Autonomous(name = "AutoBlueClose", group = "linear autoMode")

public class AutoBlueClose extends RobotLinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private double speed;
    private final int oneRotation = 540;
    private int newLFTarget;
    private int newLBTarget;
    private int newRFTarget;
    private int newRBTarget;

    @Override
    public void runOpMode() {
        declareHardwareProperties();
        waitForStart();
        encoderDrive(0.5, 20, MOVEMENT_DIRECTION.FORWARD);
        encoderDrive(0.5, 20, MOVEMENT_DIRECTION.REVERSE);
        encoderDrive(0.6, 35, MOVEMENT_DIRECTION.STRAFE_LEFT);
        encoderDrive(0.5, 5, MOVEMENT_DIRECTION.STRAFE_RIGHT);
    }
}

