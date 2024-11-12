package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robotParts.Claw;
import org.firstinspires.ftc.teamcode.robotParts.DriveTrain;
import org.firstinspires.ftc.teamcode.robotParts.LinearLift;
import org.firstinspires.ftc.teamcode.robotParts.Turn;


@TeleOp(name="DriveTrainTest", group="Linear OpMode")
@Config
public class DriveTrainTest extends LinearOpMode {
    TelemetryPacket packet = new TelemetryPacket();
    FtcDashboard dashboard = FtcDashboard.getInstance();
    public static double distance;
    public static double setpoint;
    public static double angle;
    public static double[] PidConstantsAngle = new double[]{1, 200, 0};
    private final double[] PidConstantsDistance = new double[]{0.0005, 0.01, 0};

    private final DriveTrain dt = new DriveTrain();
    private final LinearLift lin = new LinearLift();
    private final Turn turn = new Turn();
    private final Claw claw = new Claw();

    @Override
    public void runOpMode() {
        dt.init(hardwareMap);
        lin.init(hardwareMap);
        turn.init(hardwareMap);
        claw.init(hardwareMap);
//        boolean toggle = true;
        waitForStart();
        while (opModeIsActive()) {
//            if(true){
//                dt.fieldCentricDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1);
//            }else{
//                dt.robotCentricDrive(gamepad1.left_stick_y, gamepad1.left_stick_x,gamepad1.right_stick_x);
//            }
//            if(gamepad1.start){
//                toggle = !toggle;
//            }
            dt.fieldCentricDrive(gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_stick_x, 1);
            if(gamepad1.a){
                angle = dt.getImu().getAngularOrientation().firstAngle;
            }
            if(gamepad1.b){
                dt.fixAngle(PidConstantsAngle, angle);
            }
            if(gamepad1.right_bumper){
                dt.incSpeed(0.02);
            }
            if(gamepad1.left_bumper){
                dt.incSpeed(-0.02);
            }
            if(gamepad1.dpad_left){
                dt.reInitFieldCentric();
                lin.reInit();
            }
            if(gamepad2.left_stick_y!=0 && turn.getPos() >= -100){
                lin.moveLift(gamepad2.left_stick_y);
            }else{
                lin.setPower(0);
            }
            if(gamepad2.right_stick_y!=0 && lin.getPos() <= 150){
                turn.turn(gamepad2.right_stick_y);
            }else{
                turn.setPower(0);
            }

            if (gamepad2.right_bumper) {
                claw.setClaw(.8);
            }
            if (gamepad2.right_trigger != 0){
                claw.setRotate(claw.getRotate() - .005);
            }
            if (gamepad2.left_bumper) {
                claw.setClaw(.5);
            }
            if(gamepad2.left_trigger != 0){
                claw.setRotate(claw.getRotate() + .005);
            }
            if(gamepad2.x){
                lin.gotoPosition(setpoint);
            }
            if(gamepad2.y){
                turn.gotoMaxPosition(1);
            }
            packet.put("Angle", dt.getImu().getAngularOrientation().firstAngle);
            packet.put("X", dt.getxOdom().getCurrentPosition());
            packet.put("Y", dt.getyOdom().getCurrentPosition());
            packet.put("Lin", lin.getPos());
            packet.put("Turn", turn.getPos());
            packet.put("Speed", dt.getSpeed());
            packet.put("Claw Rotation", claw.getRotate());
            packet.put("Claw Open", claw.getPos());

            dashboard.sendTelemetryPacket(packet);
        }
    }
}

