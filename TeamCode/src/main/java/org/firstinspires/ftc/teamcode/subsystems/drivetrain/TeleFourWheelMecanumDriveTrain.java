package org.firstinspires.ftc.teamcode.subsystems.drivetrain;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.subsystems.vision.LimeLight;

/**
 * four wheel mecanum drive train for teleop that will drive the robot in periodic call
 */
public class TeleFourWheelMecanumDriveTrain extends FourWheelMecanumDrive {
    protected LimeLight limeLight;

    @Override
    public void periodic() {
        super.periodic();

        drive.driveRobotCentric(
                directionFlag * powerRatio * gamepad.getLeftX(),
                directionFlag * powerRatio * gamepad.getLeftY(),
                powerRatio * gamepad.getRightX() * -1
        );
    }

    public TeleFourWheelMecanumDriveTrain(HardwareMap hardwareMap, GamepadEx gamepad, Telemetry telemetry, DriverFeedback feedback, LimeLight limeLight) {
        super(hardwareMap, gamepad, telemetry, feedback, false);
        this.limeLight = limeLight;
    }

    public void AlignTx() {
        LimeLight.LimelightResult result = limeLight.GetResult();
        double min = 0.3;


        if(result != null) {
            double tx = result.getTx();
            telemetry.addData("tx", tx);

            while(Math.abs(tx) > 1.0) {

                double power = tx * -0.03;
                if(Math.abs(power) < min) {
                    power = min * Math.signum(power);
                }
                telemetry.addData("turn power", tx * -0.01);
                drive.driveRobotCentric(0, 0, power);

                result = limeLight.GetResult();

                if(result == null) {
                    break;
                }

                tx = result.getTx();
            }

            drive.stop();
        }
        else {
            telemetry.addLine("No sample found");
            telemetry.update();
        }
    }

    public void AlignTy() {
        LimeLight.LimelightResult result = limeLight.GetResult();
        double min = 0.3;

        if(result != null) {
            double ty = result.getTy() + 3.0;

            while(Math.abs(ty) > 1) {

                double power = ty * -0.02;
                if(Math.abs(power) < min) {
                    power = min * Math.signum(power);
                }
                drive.driveRobotCentric(0, power, 0);

                result = limeLight.GetResult();

                if(result == null) {
                    telemetry.addLine("cannot find sample");
                    break;
                }

                ty = result.getTy();
            }

            drive.stop();
        }
    }

}
