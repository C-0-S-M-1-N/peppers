package com.example.lib;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        double middlePurple_x = 21.5, middlePurple_y = -1.3,
                middleYellow_x = 25.24, middleYellow_y = -24.5,
                stack_x = 24, stack_y = 76.5,
                prebackdropMiddle_x = 25.24, prebackdropMiddle_y = -6,
                leftpurple_x = 14.5, leftpurple_y = 7, leftpurple_h = 0.44,
                leftyellow_x = 27, leftyellow_y = -24, leftyellow_h = Math.toRadians(-70),
                transit_x = 2, transit_y = 2, transit_h = Math.toRadians(-90),
                angledStack_x = 16, angledStack_y = 77, angledStack_h = 4;

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(65, 55, 5, Math.toRadians(180), 13.4)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(angledStack_x, angledStack_y, angledStack_h))
                                .splineToSplineHeading(new Pose2d(transit_x, transit_y + 48, transit_h), transit_h)
                                .splineToSplineHeading(new Pose2d(transit_x, transit_y + 30, transit_h), transit_h)
                                .splineToSplineHeading(new Pose2d(transit_x, transit_y, transit_h), transit_h)
                                .splineToSplineHeading(new Pose2d(leftyellow_x-15, leftyellow_y, Math.toRadians(-50)), Math.toRadians(-50))
                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}