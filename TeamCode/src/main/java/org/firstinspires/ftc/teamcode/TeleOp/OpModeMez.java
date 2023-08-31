package org.firstinspires.ftc.teamcode.TeleOp;


import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesOrder.XYZ;
import static org.firstinspires.ftc.robotcore.external.navigation.AxesReference.EXTRINSIC;
import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.util.VuforiaNav;

@TeleOp(name = "MezCamera", group = "Concept")
public class OpModeMez extends OpMode
{
    private int counter = 0;

    protected VuforiaNav mVufoiraNav;

    @Override
    public void init()
    {
        mVufoiraNav = new VuforiaNav(this.hardwareMap);
        telemetry.addData("initialized", "");
        telemetry.update();

        mVufoiraNav.start();
    }


    private static final float mmPerInch        = 25.4f;
    @Override
    public void loop()
    {

//        try
//        {

            if (mVufoiraNav.isTargetVisible())
            {
                telemetry.addData("Visible Target", "yes");
                mVufoiraNav.getTranslation();

                // express position (translation) of robot in inches.
                VectorF translation = mVufoiraNav.getTranslation();
                telemetry.addData("Pos (inches)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation.get(0) / mmPerInch, translation.get(1) / mmPerInch, translation.get(2) / mmPerInch);

                // express the rotation of the robot in degrees.
                Orientation rotation = Orientation.getOrientation(mVufoiraNav.getLastLocation(), EXTRINSIC, XYZ, DEGREES);
                telemetry.addData("Rot (deg)", "{Roll, Pitch, Heading} = %.0f, %.0f, %.0f", rotation.firstAngle, rotation.secondAngle, rotation.thirdAngle);

            }
            else
            {
                telemetry.addData("Visible Target", "none");
            }


            //sleep(1000);
//            telemetry.addData("loop", "counter" + counter++ + " locX = " + mVufoiraNav.getLocX());
            telemetry.update();
//        }
//        catch (InterruptedException e)
//        {
//            e.printStackTrace();
//        }
    }
}

