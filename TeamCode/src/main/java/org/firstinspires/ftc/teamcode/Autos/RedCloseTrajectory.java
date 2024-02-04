package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Config
public class RedCloseTrajectory {
    private TrajectorySequence preloadSeqL, preloadSeqM, preloadSeqR, lbackdropSeq ,mbackdropSeq, rbackdropSeq, parkSeq, transitSeq;

    public static double init_x = 0 , init_y = 0, init_heading = 0;
    public static double LPreload_x = -20, LPreload_y = 20, LPreload_heading = -90;
    public static double MPreload_x = 0, MPreload_y = 40, MPreload_heading = 0;
    public static double RPreload_x = 27, RPreload_y = -7, RPreload_heading = 304;

    public static double LBackdrop_x = -20, LBackdrop_y = 20, LBackdrop_heading = -90;
    public static double MBackdrop_x = 0, MBackdrop_y = 40, MBackdrop_heading = 0;
    public static double RBackdrop_x = 32.8, RBackdrop_y = 36.2, RBackdrop_heading = 91;
    public static double Park_x = 2, Park_y = 30, Park_heading = 91;
    public static double Transit_x = 11, Transit_y = -2, Transit_heading = 0.1;     
    private final Pose2d preloadPosL = new Pose2d(LPreload_x,LPreload_y,Math.toRadians(LPreload_heading));
    private Pose2d preloadPosM = new Pose2d(MPreload_x,MPreload_y,Math.toRadians(MPreload_heading));
    private Pose2d preloadPosR = new Pose2d(RPreload_x,RPreload_y,Math.toRadians(RPreload_heading));
    private Pose2d initPos = new Pose2d(init_x, init_y, Math.toRadians(init_heading));

    private Pose2d LbackdropPos = new Pose2d(LBackdrop_x,LBackdrop_y,Math.toRadians(LBackdrop_heading));
    private Pose2d MbackdropPos = new Pose2d(MBackdrop_x,MBackdrop_y,Math.toRadians(MBackdrop_heading));
    private Pose2d RbackdropPos = new Pose2d(RBackdrop_x,RBackdrop_y,Math.toRadians(RBackdrop_heading));
    private Pose2d parkPos = new Pose2d(Park_x,Park_y,Math.toRadians(Park_heading));
    private Pose2d transitPos = new Pose2d(Transit_x, Transit_y, Math.toRadians(Transit_heading));

    public RedCloseTrajectory(SampleMecanumDriveCancelable mecanumDrive) {

        preloadSeqL = mecanumDrive.trajectorySequenceBuilder(initPos)
                .lineToLinearHeading(preloadPosL)
                .build();

        preloadSeqM = mecanumDrive.trajectorySequenceBuilder(initPos)
                .lineToLinearHeading(preloadPosM)
                .build();

        preloadSeqR = mecanumDrive.trajectorySequenceBuilder(initPos)
                .lineToLinearHeading(preloadPosR)
                .build();


        lbackdropSeq = mecanumDrive.trajectorySequenceBuilder(initPos)
                .lineToLinearHeading(LbackdropPos)
                .build();
        mbackdropSeq = mecanumDrive.trajectorySequenceBuilder(initPos)
                .lineToLinearHeading(MbackdropPos)
                .build();
        rbackdropSeq = mecanumDrive.trajectorySequenceBuilder(initPos)
                .lineToLinearHeading(RbackdropPos)
                .build();

        parkSeq = mecanumDrive.trajectorySequenceBuilder(initPos)
                .lineToLinearHeading(parkPos)
                .build();

        transitSeq = mecanumDrive.trajectorySequenceBuilder(initPos)
                .lineToLinearHeading(transitPos)
                .build();
    }
    public TrajectorySequence PreloadL(){
        return preloadSeqL;
    }
    public TrajectorySequence PreloadM(){
        return preloadSeqM;
    }
    public TrajectorySequence PreloadR(){
        return preloadSeqR;
    }
    public TrajectorySequence LBackdrop(){
        return lbackdropSeq;
    }
    public TrajectorySequence MBackdrop(){
        return mbackdropSeq;
    }
    public TrajectorySequence RBackdrop(){
        return rbackdropSeq;
    }
    public TrajectorySequence Park(){
        return parkSeq;
    }
    public TrajectorySequence Transit(){return transitSeq;}
}
