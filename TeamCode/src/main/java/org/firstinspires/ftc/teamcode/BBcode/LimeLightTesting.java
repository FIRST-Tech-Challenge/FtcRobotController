package org.firstinspires.ftc.teamcode.BBcode;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.PinpointDrive;

@TeleOp(name = "LimeLightTesting")
//@Disabled
public class LimeLightTesting extends LinearOpMode {
    //classes
    MecanumDrivetrain _MecanumDrivetrain;

    //vars
    private Pose3D botPose;
    private double botYaw;

    boolean tagFound = false;

    double aError;
    double xError;
    double yError;

    double turn;
    double strafe;
    double drive;

    //Hardware
    DcMotorEx _leftFront;
    DcMotorEx _leftBack;
    DcMotorEx _rightFront;
    DcMotorEx _rightBack;
    Limelight3A _limelight;

    //settings
    final Pose3D targetPose = new Pose3D(new Position(DistanceUnit.METER, -1.203, -1.203, 0,0), new YawPitchRollAngles(AngleUnit.DEGREES, 180, 0, 0, 0));
    final double turnSpeed = 0.1;
    final double strafeSpeed = 0.5;
    final double driveSpeed = 0.5;
    final double maxTurnSpeed = 1;
    final double maxStrafeSpeed = 1;
    final double maxDriveSpeed = 1;

    @Override
    public void runOpMode() throws InterruptedException {
    //Init steps
        //create instance of external classes
        _MecanumDrivetrain = new MecanumDrivetrain(this);

        //Init limelight
        _limelight = hardwareMap.get(Limelight3A.class, "limelight");
        _limelight.pipelineSwitch(1);
        _limelight.start();

        //Init drive Motors
        _leftFront = hardwareMap.tryGet(DcMotorEx.class, "leftFront");
        _leftBack = hardwareMap.tryGet(DcMotorEx.class, "leftBack");
        _rightFront = hardwareMap.tryGet(DcMotorEx.class, "rightFront");
        _rightBack = hardwareMap.tryGet(DcMotorEx.class, "rightBack");

        _rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        _rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

    //run steps
        while (opModeIsActive()) {
            //Manage LimeLight
            LLResult lLResult = _limelight.getLatestResult();
            if (lLResult != null) {
                if (lLResult.getTa() > 0) {
                    tagFound = true;
                    botPose = lLResult.getBotpose();
                    botYaw = lLResult.getBotpose().getOrientation().getYaw();
                    telemetry.addData("BotPose", botPose.getPosition());
                    telemetry.addData("Yaw", botYaw);
                }
                else {
                    tagFound = false;
                    telemetry.addData("Limelight", "Tag not found");
                }
            }
            else {
                tagFound = false;
                telemetry.addData("Limelight", "No data available");
            }

            //Auto drive
            if (tagFound && gamepad1.left_bumper) {
                DriveToAprilTag();
            }

            //calc pose error
            //aError
            aError = targetPose.getOrientation().getYaw() - botYaw;
            if (Math.abs(aError) > 180) {aError = -1 * (Math.signum(aError) * (180 - (Math.abs(aError) % 180)));}
            //xError
            xError = targetPose.getPosition().x - botPose.getPosition().x;
            //yError
            yError = targetPose.getPosition().y - botPose.getPosition().y;

            //Manual drive
            _MecanumDrivetrain.Drive();

            telemetry.addData("aError",aError);
            telemetry.addData("xError",xError);
            telemetry.addData("yError",yError);
            telemetry.update();
        }
        _limelight.stop();
    }

    private void DriveToAprilTag() {

        //calc movement magnitudes
        //turn
        turn = Range.clip(turnSpeed * aError, (-1 * maxTurnSpeed), maxTurnSpeed);
        //strafe
        double xCorrectionByStrafe = (xError * ((90 - Math.abs(botYaw)) / 90)); // the amount of x error corrected by strafing
        double yCorrectionByStrafe = (yError * (Math.signum(botYaw) * Math.abs((Math.abs(botYaw))) - 90) / 90);
        strafe = Range.clip(strafeSpeed * (yCorrectionByStrafe + xCorrectionByStrafe), -1 * maxStrafeSpeed, maxStrafeSpeed);
        //drive
        double xCorrectionByDrive = (xError * (Math.signum(botYaw) * Math.abs((Math.abs(botYaw))) - 90) / 90);
        double yCorrectionByDrive = (yError * ((90 - Math.abs(botYaw)) / 90));
        drive = Range.clip(driveSpeed * (yCorrectionByDrive + xCorrectionByDrive), -1 * maxDriveSpeed, maxDriveSpeed);

        //sets motor powers
        _leftFront.setPower(Range.clip((drive + turn + strafe), -1, 1));
        _leftBack.setPower(Range.clip((drive + turn - strafe), -1, 1));
        _rightFront.setPower(Range.clip((drive - turn - strafe), -1, 1));
        _rightBack.setPower(Range.clip((drive - turn + strafe), -1, 1));
    }
}