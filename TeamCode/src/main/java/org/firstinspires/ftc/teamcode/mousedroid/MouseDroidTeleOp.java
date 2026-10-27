package org.firstinspires.ftc.teamcode.mousedroid;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;

//import com.bylazar.configurables.annotations.Configurable;
//import com.bylazar.configurables.annotations.Sorter;

@TeleOp(name = "MouseDroidTeleOp")
public class MouseDroidTeleOp extends OpMode {



//    @Configurable
//    public class SortedConfigurables {
//        @Sorter(sort = 0)
//        public static double kP = 0.0;
//
//        @Sorter(sort = 1)
//        public static double kI = 0.0;
//
//        @Sorter(sort = 2)
//        public static double kD = 0.0;
//    }


TankDriveHW m_tankDrive;

PanelsTelemetry    panelsTelemetry;

    TelemetryManager panels;

    @Override
    public void init() {

        m_tankDrive = new TankDriveHW((this));

         panels =
              PanelsTelemetry.INSTANCE.getTelemetry();

        telemetry.addLine("Panels test initialized");
        telemetry.update();

        //PanelsTelemetry;
    }

    @Override
    public void loop() {

        double rightStickX = gamepad1.right_stick_x;
        double rightStickY = gamepad1.right_stick_y; //negative value when stick is pressed forward

        double leftPower = 0;
        double rightPower = 0;

        double cutSpeed = rightStickX;

        leftPower = 1.4*(rightStickY - cutSpeed);
        rightPower = -1.4*(rightStickY + cutSpeed);



        m_tankDrive.tankDrive(leftPower, rightPower);

        //panelsTelemetry.getFtcTelemetry().addLine("This is a test");

        telemetry.addData("rightStickX", gamepad1.right_stick_x);
        telemetry.addData("rightStickY", gamepad1.right_stick_y);
        telemetry.addData("leftPower", leftPower);
        telemetry.addData("rightPower", rightPower);
        telemetry.update();


        panels.addData("rightStickX", gamepad1.right_stick_x);
        panels.addData("rightStickY", gamepad1.right_stick_y);
        panels.addData("leftPower", leftPower);
        panels.addData("rightPower", rightPower);

        panels.update();

        // Send real-time data back to Panels dashboard
        //telemetry.addData("leftPower", leftPower);
        //telemetry.addData("rightPower", rightPower);
//        telemetry.addData("X", follower.pose().x());
//        telemetry.addData("Y", follower.pose().y());
//        telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));


    }

    @Override
    public void start() {
    }

}
