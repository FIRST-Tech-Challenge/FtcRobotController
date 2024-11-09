package org.firstinspires.ftc.teamcode.opmodes.autonomous.red;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.opmodes.autonomous.base.AutoOpModeBase;

@Autonomous(name = "_Red > Start F2", group = "Red alliance", preselectTeleOp="MainTeleOp")
public class Red_F3 extends AutoOpModeBase {

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void executeOpMode() {

        rollingIntake.SetElbowInIntakePosition();
        driveTrain.DriveToTarget(300, 400);
        driveTrain.TurnAngle(125);

        pivot.MoveToDeliveryInAuto();
        slider.ExtendMaxInAuto();

        driveTrain.DriveToTarget(200, 400);

        rollingIntake.OuttakeInAuto();
        Wait(500);
        rollingIntake.HoldInAuto();

        Wait(10000);

//        rollingIntake.SetElbowInIntakePosition();
//
//        rollingIntake.IntakeInAuto();
//
//        pivot.MoveToIntakeInAuto();
//
//        pivot.MoveToIntakeSampleInAuto();
//        Wait(300);
//        rollingIntake.HoldInAuto();
//
//        pivot.MoveToDeliveryInAuto();
//
//        rollingIntake.OuttakeInAuto();
//        Wait(500);
//        rollingIntake.HoldInAuto();

    }

}