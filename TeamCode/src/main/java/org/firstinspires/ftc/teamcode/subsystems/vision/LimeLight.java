package org.firstinspires.ftc.teamcode.subsystems.vision;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.subsystems.SonicSubsystemBase;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.FourWheelMecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.drivetrain.TeleFourWheelMecanumDriveTrain;
import org.firstinspires.ftc.teamcode.subsystems.feedback.DriverFeedback;
import org.firstinspires.ftc.teamcode.util.SonicPIDController;

public class LimeLight extends SonicSubsystemBase {

    private Telemetry telemetry;

    GamepadEx gamepad;

    private Limelight3A limelight;


    public LimeLight(HardwareMap hardwareMap, Telemetry telemetry) {
        /* instantiate motors */

        this.telemetry = telemetry;

        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        telemetry.setMsTransmissionInterval(11);

        limelight.pipelineSwitch(4);

        limelight.start();
    }

    @Override
    public void periodic() {
        super.periodic();

        telemetry.addData("source", "limelight");

        if(limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    Pose3D botpose = result.getBotpose();
//                    telemetry.addData("tx", result.getTx());
//                    telemetry.addData("ty", result.getTy());
//                    telemetry.addData("Botpose", botpose.toString());
                }
            }
        }
    }

    public class LimelightResult {

        public LimelightResult(double tx, double ty) {
            this.tx = tx;
            this.ty = ty;
        }

        private double tx, ty;

        public double getTx() {
            return tx;
        }

        public double getTy() {
            return ty;
        }
    }

    public LimelightResult GetResult() {
        if(limelight != null) {
            LLResult result = limelight.getLatestResult();
            if (result != null) {
                if (result.isValid()) {
                    telemetry.addData("tx", result.getTx());
                    telemetry.addData("ty", result.getTy());

                    return new LimelightResult(result.getTx(), result.getTy());
                }
            }
        }

        return null;
    }
}