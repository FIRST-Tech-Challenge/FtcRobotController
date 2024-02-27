package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.robotParts.DriveTrain;
import org.firstinspires.ftc.teamcode.robotParts.LinearLift;


@TeleOp(name="DriveTrainTest", group="Linear OpMode")
@Config
public class DriveTrainTest extends LinearOpMode {
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double distance;
    public static double setpoint;
    public static double angle;
    private final double[] PidConstantsAngle = new double[]{1, 200, 0};
    private final double[] PidConstantsDistance = new double[]{0.0005, 0.01, 0};

    private final DriveTrain dt = new DriveTrain();
    private final LinearLift lin = new LinearLift();

    @Override
    public void runOpMode() {
        dt.init(hardwareMap);
        lin.init(hardwareMap);
        boolean toggle = false;
        waitForStart();
        while (opModeIsActive()) {
            if(toggle){
                dt.FieldCentricDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1);
            }else{
                dt.robotCentricDrive(gamepad1.left_stick_y, gamepad1.left_stick_x,gamepad1.right_stick_x);
            }
            if(gamepad1.start){
                toggle = !toggle;
            }
            if(gamepad1.a){
                dt.getToAngle(PidConstantsAngle, angle);
            }
            if(gamepad1.b){
                dt.driveToLocation(PidConstantsDistance, angle,distance);
            }
            if(gamepad1.back && gamepad1.dpad_up){
                dt.reInitFieldCentric();
            }
            if(gamepad2.left_stick_y!=0){
                lin.moveLift(gamepad2.left_stick_y);
            }else{
                lin.setPower(0);
            }
            if(gamepad2.a){
                lin.gotoPosition(setpoint);
            }
            packet.put("Angle", dt.getImu().getAngularOrientation().firstAngle);
            packet.put("X", dt.getxOdom().getCurrentPosition());
            packet.put("Y", dt.getyOdom().getCurrentPosition());
            packet.put("Lin", lin.getPos());
            dashboard.sendTelemetryPacket(packet);
        }
    }
}

