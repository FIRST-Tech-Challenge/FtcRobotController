package org.firstinspires.ftc.teamcode.robotAttachments;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.robotAttachments.odometry.Executable;
import org.firstinspires.ftc.teamcode.robotAttachments.odometry.OdometryGlobalCoordinatePosition;
import org.firstinspires.ftc.teamcode.robotAttachments.odometry.OdometryMovement;

public class AutonomousDriveSystem {
    DcMotor front_right;
    DcMotor front_left;
    DcMotor back_right;
    DcMotor back_left;

    DcMotor horizontalEncoder;
    DcMotor verticalLeftEncoder;
    DcMotor verticalRightEncoder;

    Executable<Boolean> isStopRequested;
    Executable<Boolean> isOpModeActive;
    Telemetry telemetry;


    public AutonomousDriveSystem(HardwareMap hardwareMap,
                                 String front_right,
                                 String front_left,
                                 String back_right,
                                 String back_left,
                                 String horizontal_encoder,
                                 String leftVerticalEncoder,
                                 String rightVerticalEncoder,
                                 String horizontalServo,
                                 String leftVerticalServo,
                                 String rightVerticalServo,
                                 OdometryGlobalCoordinatePosition odometry;

    ) {
        if(horizontal_encoder == front_right){
            this.front_right = hardwareMap.dcMotor.get(front_right);
            this.horizontalEncoder = this.front_right;
        }

        this.front_left = hardwareMap.dcMotor.get(front_left);
        this.back_right = hardwareMap.dcMotor.get(back_right);
        this.back_left = hardwareMap.dcMotor.get(back_left);



        }
        Executable<Boolean> isStopRequested;
        Executable<Boolean> isOpModeActive;
        Telemetry telemetry;

        OdometryMovement odMovement = new OdometryMovement(front_right, front_left, back_right, back_left, telemetry, odometry, isStopRequested, isOpModeActive, )

    }
}
