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
        DeliverSample();

        driveTrain.DriveToTarget(400, 300);
        Wait(300);
        driveTrain.TurnAngle(0);

        driveTrain.AlignTx();
        driveTrain.AlignTy();

        driveTrain.TurnRelative(5);
        driveTrain.Forward(120);

        pivot.MoveToIntakeInAuto();

        rollingIntake.IntakeInAuto();
        Wait(300);

        pivot.MoveToIntakeSampleInAuto();
        Wait(500);

        rollingIntake.HoldInAuto();

        pivot.MoveToDeliveryInAuto();

        DeliverSample();

        driveTrain.DriveToTarget(1500, 0);

    }

    private void DeliverSample() {
        driveTrain.DriveToTarget(270, 400);
        driveTrain.TurnAngle(125);

        pivot.MoveToDeliveryInAuto();
        slider.ExtendMaxInAuto();

        driveTrain.DriveToTarget(80, 400);

        rollingIntake.OuttakeInAuto();
        Wait(00);
        rollingIntake.HoldInAuto();

        driveTrain.DriveToTarget(300, 400);
        slider.CollapseMinInAuto();
        pivot.MoveToStartInAuto();
    }

}