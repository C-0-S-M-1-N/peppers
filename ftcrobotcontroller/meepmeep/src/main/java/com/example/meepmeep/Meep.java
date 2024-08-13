package com.example.meepmeep;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.SampleMecanumDrive;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

import javax.swing.plaf.basic.BasicOptionPaneUI;

public class Meep {
    public static double x = 14, y = 62, rot = -90;

    public static Pose2d
            MiddlePurple = new Pose2d(24, -1, 0),
            MiddleYellow = new Pose2d(21, -25, Math.toRadians(286)),

    RightPurple = new Pose2d(44.7, -5, Math.toRadians(190)),
            RightYellow = new Pose2d(37, -32, Math.toRadians(233)),

    LeftPurple = new Pose2d(12, 8, Math.toRadians(33)),
            LeftYellow = new Pose2d(42, -30, Math.toRadians(245));
    public static Pose2d
            TrussToStack     = new Pose2d(3, 45, -Math.PI/2.f),
            Stack            = new Pose2d(22, 75, -Math.toRadians(110)),
            Stack2           = new Pose2d(34, 75, -Math.toRadians(110)),
            BackBoardToTruss = new Pose2d(3, 9, -Math.PI/2.f),
            Backdrop         = new Pose2d(14, -27.5, -Math.toRadians(80)),
            TrussToStack_s     = new Pose2d(3, 45, -Math.PI/2.f),
            BackBoardToTruss_s = new Pose2d(3, 9, -Math.PI/2.f);

    public static void main(String[] args){
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity bot = new DefaultBotBuilder(meepMeep)
                .setConstraints(70, 55, 5, 5, 12.2)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(MiddleYellow)
                                .setReversed(true)

                                .splineToSplineHeading(BackBoardToTruss, Math.toRadians(90))
                                .splineToSplineHeading(TrussToStack, Math.toRadians(90))
                                .splineToSplineHeading(Stack, Math.toRadians(50))
                                .lineToLinearHeading(Stack2)
                                .setReversed(false)
                                .lineToLinearHeading(new Pose2d(20, Stack.getY() - 5, Stack.getHeading()))
                                .splineToSplineHeading(TrussToStack_s, Math.toRadians(-90))
                                .splineToSplineHeading(BackBoardToTruss, Math.toRadians(-90))
                                .splineToSplineHeading(Backdrop, Math.toRadians(-85))
                                .build()


                );
        meepMeep.setBackground(MeepMeep.Background.FIELD_CENTERSTAGE_JUICE_DARK)
                .setDarkMode(false)
                .setBackgroundAlpha(0.95f)
                .addEntity(bot)
                .start();
    }
}