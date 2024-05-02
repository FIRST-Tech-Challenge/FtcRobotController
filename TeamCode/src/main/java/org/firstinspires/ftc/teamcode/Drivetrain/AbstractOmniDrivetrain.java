package org.firstinspires.ftc.teamcode.Drivetrain;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

public abstract class AbstractOmniDrivetrain extends AbstractDrivetrain {


    double impulseRotation;
    public AbstractOmniDrivetrain(DcMotor FLM, DcMotor FRM, DcMotor BLM, DcMotor BRM, double impulseRotation){
        super(FLM, FRM, BLM, BRM);

        this.impulseRotation = impulseRotation;

    }

    public void setPowerBasic(double power){
        for(DcMotor motor : driveMotors){
            motor.setPower(power);
        }
    }
    public void mecanumDrive(double drive, double strafe, double turn, double heading){

        drive = ((drive * Math.cos(heading)) + ( strafe * Math.sin(heading)));
        strafe = ((drive * -Math.sin(heading)) + (strafe * Math.cos(heading)));

        double[] speeds = {
                (drive + strafe + turn),
                (drive - strafe - turn),
                (drive - strafe + turn),
                (drive + strafe - turn)
        };

        double max = Math.abs(speeds[0]);
        for (double speed : speeds) {
            if (max < Math.abs(speed)) max = Math.abs(speed);
        }

        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        driveMotors[0].setPower(speeds[0]);
        driveMotors[1].setPower(speeds[1]);
        driveMotors[2].setPower(speeds[2]);
        driveMotors[3].setPower(speeds[3]);

    }

}
