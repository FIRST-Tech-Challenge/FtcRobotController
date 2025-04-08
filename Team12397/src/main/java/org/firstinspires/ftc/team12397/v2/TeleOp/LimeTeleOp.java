package org.firstinspires.ftc.team12397.v2.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.team12397.v2.LLtdc;
import org.firstinspires.ftc.team12397.v2.RobotHardware;
import org.firstinspires.ftc.team12397.v2.TdcReturnObject;
@TeleOp(name="LimeTeleOp", group="Robot")

public class LimeTeleOp extends LinearOpMode {

    Limelight3A limelight = this.hardwareMap.get(Limelight3A.class, "limelight-rfc");

    LLtdc tdc = new LLtdc(limelight);

    TdcReturnObject tdcReturn;

    RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {
        double drive = 0;
        double strafe = 0;
        double turn = 0;


        Gamepad luisL = gamepad1;
        Gamepad alexH = gamepad2;

        tdc.initialize(telemetry);
        robot.init();
        telemetry.addData(">", "Hardware Initialized");

        double testVar = 0;

        while (opModeIsActive()) {
            telemetry.clear();


            if (luisL.a){
                tdc.assessEnvironment(0);
                tdcReturn = tdc.getTdcReturn();
            }
            telemetry.addData("Scan successful? ", tdc.getScanSuccess());
            telemetry.addData("Inches right/left", tdcReturn.getRobotXCorrection(DistanceUnit.INCH));
            telemetry.addData("Inches right/left", tdcReturn.getRobotYCorrection(DistanceUnit.INCH));
            telemetry.addData("Inches right/left", tdcReturn.getArmYCorrection(DistanceUnit.INCH));
            telemetry.addData("rads cw/ccw", tdcReturn.getYawCorrection(AngleUnit.DEGREES));
            telemetry.addData("rads cw/ccw", tdcReturn.getClawYawCorrection(AngleUnit.DEGREES));
            telemetry.update();

            if (tdc.getScanSuccess()){
                testVar = tdcReturn.getYawCorrection(AngleUnit.RADIANS)/ (2* Math.PI);
                robot.setServoPosition(0, testVar);
            }
        }
    }


}
