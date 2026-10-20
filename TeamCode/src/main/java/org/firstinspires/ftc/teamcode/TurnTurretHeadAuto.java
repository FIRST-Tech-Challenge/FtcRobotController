package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCompatibilityManager;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.internal.usb.UsbConstants;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagClusterDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

public class TurnTurretHeadAuto {
    private boolean shotAvalible;
    private MotorPowerController turretHead;
    private CameraControl camera;

    public TurnTurretHeadAuto(HardwareMap hm, String turretHeadName) {
        turretHead = new MotorPowerController(hm, turretHeadName);
        shotAvalible = false;
        camera = new CameraControl();
    }

    public boolean moveHead() {
        while(!shotAvalible) {
            if (camera.isGreen()){
                shotAvalible = true;
            }
            turretHead.setPower(1);
        }
        shotAvalible = false;
    }
}
