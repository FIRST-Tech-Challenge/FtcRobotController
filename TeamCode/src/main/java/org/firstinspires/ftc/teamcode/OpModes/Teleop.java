package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;


import org.ejml.simple.SimpleMatrix;
import org.firstinspires.ftc.teamcode.Drivetrain.Drivetrain;


@Config
@TeleOp(name = "TeleOp", group = "Autonomous")
public class Teleop extends LinearOpMode {
    Drivetrain drivetrain = null;
    // Use FTCDashboard
    FtcDashboard dashboard;
    @Override
    public void runOpMode(){
        drivetrain = new Drivetrain(hardwareMap);
        dashboard = FtcDashboard.getInstance();
        telemetry = dashboard.getTelemetry();
        TelemetryPacket packet = new TelemetryPacket();
        waitForStart();
        if (isStopRequested()) return;
        while(opModeIsActive()){
            double y = -gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y - x + rx) / denominator;
            double backLeftPower = (y + x + rx) / denominator;
            double frontRightPower = (y + x - rx) / denominator;
            double backRightPower = (y - x - rx) / denominator;

            SimpleMatrix drivePowers = new SimpleMatrix(
                    new double[][]{
                            new double[]{frontLeftPower},
                            new double[]{backLeftPower},
                            new double[]{backRightPower},
                            new double[]{frontRightPower}
                    }
            );
            drivetrain.setPower(drivePowers);
            packet.fieldOverlay()
                    .setFill("white")
                    .fillRect(0,0,10, 10);
//                    .drawGrid(-72, -72, 72, 72, 12, 12);
//                    .drawImage("data:image/png;base64,iVBORw0KGgoAAAANSUhEUgAAAJQAAACUCAMAAABC4vDmAAAAPFBMVEX+/v7///8hHR6lpKUAAAD6+vrOzs6XlZZWU1QtKSrW1tZta2x0cnKampqsrKxPTU3Dw8OEg4Pv7u4NAAXZd1g3AAABLElEQVR4nO3cOZKEMBAFUUmIfafvf9cBIsZPjzIyjYoy3wl+yk9t1w9j+bhx6Lv25aTnTHMty9p83LqUOk//qK3ux4v7unTsdcsvavud1/0EKOfr/G0PaqpnDNJTzmedcmrn/QpjulXXPrepq0cg0606apf6kmKhUunTsIQy3aplSOMaDbWOqTTRUE0RhRJFE0UTRRNFE0UTRRNFE0UTRRNFE0UTRRNFE0UTRRNFE0UTRRNFE0UTRRNFE0UTRRNFE0UTRRNFE0UTRRNFE0UTRRNFE0UTRRNFE0UTRRNFE0UTRRNFE0UTRRNFE0UTRRNFE0UTRRNFE0UTRRNFE0UTRRNFE0UTRRNFE0ULiwo5ihVyPizk0FrISbqQ430hZw5jDkLGnM6MOTIabo71D8yDEg3YNJqVAAAAAElFTkSuQmCC", drivetrain.state.get(0,0), drivetrain.state.get(1,0), 16 / 12, 17 / 12);
            dashboard.sendTelemetryPacket(packet);

        }
    }
}
