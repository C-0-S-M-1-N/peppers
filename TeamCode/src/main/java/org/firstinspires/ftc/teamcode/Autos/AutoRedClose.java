package org.firstinspires.ftc.teamcode.Autos;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.OutTake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;

@Config
@Autonomous(group = "Autos", preselectTeleOp = "pipers \\uD83C\\uDF36", name = "AutoRedClose")
public class AutoRedClose extends LinearOpMode {
    private org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive mecanumDrive;

    private TrajectorySequence preloadSeqL, preloadSeqM, preloadSeqR, backdropSeq, parkSeq;
    public static double LPreload_x,LPreload_y,LPreload_heading;
    public static double MPreload_x,MPreload_y,MPreload_heading;
    public static double RPreload_x,RPreload_y,RPreload_heading;
    public static double Backdrop_x,Backdrop_y,Backdrop_heading;
    public static double Park_x,Park_y,Park_heading;
    private Pose2d preloadPosL = new Pose2d(LPreload_x,LPreload_y,LPreload_heading) ,
            preloadPosM = new Pose2d(MPreload_x,MPreload_y,MPreload_heading) ,
            preloadPosR = new Pose2d(RPreload_x,RPreload_y,RPreload_heading) ,
            backdropPos = new Pose2d(Backdrop_x,Backdrop_y,Backdrop_heading) ,
            parkPos = new Pose2d(Park_x,Park_y,Park_heading);
    org.firstinspires.ftc.teamcode.Autonomy.ObjectCaseDetection detection = new org.firstinspires.ftc.teamcode.Autonomy.ObjectCaseDetection(hardwareMap,telemetry);

    private boolean updateMecanum = true, reachedBackdrop = false, retreatElevator = false;
    @Override
    public void runOpMode() throws InterruptedException {
        Controls c = new Controls(gamepad1, gamepad2);
        mecanumDrive = new SampleMecanumDrive(hardwareMap);

        OutTake outTake = new OutTake(hardwareMap, telemetry);
        Intake intake = new Intake();

        preloadSeqL = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(preloadPosL)
                .build();

        preloadSeqM = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(preloadPosM)
                .build();

        preloadSeqR = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(preloadPosR)
                .build();


        backdropSeq = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(backdropPos)
                .build();

        parkSeq = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(parkPos)
                .build();

        switch (detection.getLocation()){
            case MIDDLE:
                mecanumDrive.followTrajectorySequenceAsync(preloadSeqM);
                break;
            case RIGHT:
                mecanumDrive.followTrajectorySequenceAsync(preloadSeqR);
                break;
            default:
                mecanumDrive.followTrajectorySequenceAsync(preloadSeqL);
                break;
        }
        while(opModeIsActive() && !isStopRequested()){
            if(!mecanumDrive.isBusy()){
                mecanumDrive.followTrajectorySequenceAsync(backdropSeq);
                updateMecanum = true;
            }
            if(mecanumDrive.getPoseEstimate() == backdropPos){
                if(!reachedBackdrop){
                    OutTake.STATES.currentLevel = 2;
                    Controls.ExtendElevator = true;
                }
                updateMecanum = false;
                if(outTake.STATE == OutTake.STATES.IDLE && reachedBackdrop){
                    Controls.DropLeft = true;
                    Controls.DropRight = true;
                    updateMecanum = true;
                    retreatElevator = true;
                }
                reachedBackdrop = true;
            }
            if(retreatElevator == true)
            {
                OutTake.STATES states = OutTake.STATES.RETRACT_TRIGGER;
                mecanumDrive.followTrajectorySequenceAsync(parkSeq);
            }
            if(updateMecanum)
                mecanumDrive.update();
            intake.update();
            outTake.update();
            c.loop();
        }

    }
}
