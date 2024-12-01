package org.bluebananas.ftc.roadrunneractions.TrajectoryActionBuilders;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import java.util.function.Function;

public class Blue_Basket_Trajectories {
//set positions
    
    public static Action BlueBasket(Function<Pose2d, TrajectoryActionBuilder> builderFunction)
    {
        Vector2d basket_clear_position = new Vector2d(42, 42);
        double basket_clear_heading = Math.toRadians(45);

        Vector2d drop_position = new Vector2d(53, 53);
        double drop_heading = Math.toRadians(45);

        Vector2d outer_sample_pickup_position = new Vector2d(45, 40);
        double sample_pickup_heading = Math.toRadians(-90);

        Pose2d startPose = new Pose2d(31, 63, Math.toRadians(0));
        TrajectoryActionBuilder builder = builderFunction.apply(startPose);
        return builder
                .strafeToLinearHeading(basket_clear_position, basket_clear_heading)
                .waitSeconds(3)//bring up arm
                .splineTo(drop_position, drop_heading)
                .waitSeconds(0.5)//deposit sample
                .lineToXConstantHeading(basket_clear_position.x)
                .waitSeconds(2)//bring down arm
                .splineTo(outer_sample_pickup_position, sample_pickup_heading)
                .waitSeconds(2)//pick up sample
                .strafeToLinearHeading(basket_clear_position, basket_clear_heading)
                .waitSeconds(3)//bring up arm
                .splineTo(drop_position, drop_heading)
                .waitSeconds(0.5)//deposit sample
                .lineToXConstantHeading(basket_clear_position.x)
                .waitSeconds(2)//bring down arm
                .strafeToLinearHeading(outer_sample_pickup_position.plus(new Vector2d(10.5, 0)), sample_pickup_heading)
                .waitSeconds(2)//pick up sample
                .strafeToLinearHeading(basket_clear_position, basket_clear_heading)
                .waitSeconds(3)//bring up arm
                .splineTo(drop_position, drop_heading)
                .waitSeconds(0.5)//deposit sample
                .lineToXConstantHeading(basket_clear_position.x)
                .waitSeconds(2)//bring down arm
                .splineTo(new Vector2d(24,60), Math.toRadians(180))
                .strafeTo(new Vector2d(-36,60))
                .build();
    }

}
