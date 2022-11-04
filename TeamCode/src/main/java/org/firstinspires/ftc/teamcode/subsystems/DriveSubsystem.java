package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaCurrentGame;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.dragonswpilib.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

    private final Telemetry mTelemetry;
    private final HardwareMap mHardwareMap;

    private final DcMotor mLeftMotor;
    private final DcMotor mRightMotor;
    private final DcMotor mBackMotor;

    private double mX = 0;
    private double mY = 0;
    private double mZ = 0;

    private final VuforiaCurrentGame mVuforiaPOWERPLAY;
    private VuforiaBase.TrackingResults mVuforiaResults;
    // Pour suivre la position sur le terrain. Donnée par Vuforia.
    private double mPositionX = 0;
    private double mPositionY = 0;
    private double mRotationZ = 0;

    public DriveSubsystem(HardwareMap hardwareMap, Telemetry telemetry) {
        mTelemetry = telemetry;
        mHardwareMap = hardwareMap;

        mLeftMotor = mHardwareMap.get(DcMotor.class, "left motor");
        mRightMotor = mHardwareMap.get(DcMotor.class, "right motor");
        mBackMotor = mHardwareMap.get(DcMotor.class, "back motor");

        mLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        mRightMotor.setDirection(DcMotor.Direction.REVERSE);
        mBackMotor.setDirection(DcMotor.Direction.REVERSE);

        mLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        mBackMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        mBackMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize Vuforia
        mVuforiaPOWERPLAY = new VuforiaCurrentGame();
        // Initialize using external web camera.
        mVuforiaPOWERPLAY.initialize(
                "", // vuforiaLicenseKey
                mHardwareMap.get(WebcamName.class, "Webcam 1"), // cameraName
                "", // webcamCalibrationFilename
                false, // useExtendedTracking
                true, // enableCameraMonitoring
                VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES, // cameraMonitorFeedback
                0, // dx
                0, // dy
                0, // dz
                AxesOrder.XZY, // axesOrder
                90, // firstAngle
                90, // secondAngle
                0, // thirdAngle
                true); // useCompetitionFieldTargetLocations
        // Activate here for camera preview.
        mVuforiaPOWERPLAY.activate();

    }

    @Override
    protected void finalize() {
        // Don't forget to deactivate Vuforia before the garbage collector removes the DriveSubsystem from memory
        mVuforiaPOWERPLAY.deactivate();
        mVuforiaPOWERPLAY.close();
    }

    private void holonomicDrive (double x, double y, double z)
    {
        double backPower = Range.clip(-x + z, -1.0, 1.0);
        double leftPower = Range.clip((x / 2) + (y * Math.sqrt(3) / 2) + z, -1.0, 1.0);
        double rightPower = Range.clip((x / 2) + (-(y * (Math.sqrt(3) / 2))) + z, -1.0, 1.0);

        mLeftMotor.setPower(leftPower);
        mRightMotor.setPower(rightPower);
        mBackMotor.setPower(backPower);
    }

    @Override
    public void periodic() {
        // Are the targets visible?
        // (Note we only process first visible target).
        if (isTargetVisible("Red Audience Wall")) {
            processTarget();
        } else if (isTargetVisible("Red Rear Wall")) {
            processTarget();
        } else if (isTargetVisible("Blue Audience Wall")) {
            processTarget();
        } else if (isTargetVisible("Blue Rear Wall")) {
            processTarget();
        } else {
            mTelemetry.addData("No Targets Detected", "Targets are not visible.");
        }

        mTelemetry.addData("X (in)", Double.parseDouble(JavaUtil.formatNumber(mPositionX, 2)));
        mTelemetry.addData("Y (in)", Double.parseDouble(JavaUtil.formatNumber(mPositionY, 2)));
        mTelemetry.addData("Rotation about Z (deg)", Double.parseDouble(JavaUtil.formatNumber(mRotationZ, 2)));

        holonomicDrive(mX, mY, mZ);
    }

    public void drive(double x, double y, double z){
            mX = x;
            mY = y;
            mZ = z;
    }

    public void stop () {
        drive(0, 0, 0);
    }

    /**
     * Check to see if the target is visible.
     */
    private boolean isTargetVisible(String trackableName) {
        // Get vuforia results for target.
        mVuforiaResults = mVuforiaPOWERPLAY.track(trackableName);
        return mVuforiaResults.isVisible;
    }

    /**
     * This function displays location on the field and rotation about the Z
     * axis on the field. It uses results from the isTargetVisible function.
     */
    private void processTarget() {
        // Display the target name.
        mTelemetry.addData("Target Detected", mVuforiaResults.name + " is visible.");

        mPositionX = mVuforiaResults.x/Constants.DriveConstants.kInchToMm;
        mPositionY = mVuforiaResults.y/Constants.DriveConstants.kInchToMm;
        mRotationZ = mVuforiaResults.zAngle;
    }

    public  double getPositionX(){
        return  mPositionX;
    }

}

