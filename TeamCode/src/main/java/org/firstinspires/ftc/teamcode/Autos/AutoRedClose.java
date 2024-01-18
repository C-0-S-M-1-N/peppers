/*package org.firstinspires.ftc.teamcode.Autos;


import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.OutTake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config

public class AutoRedClose extends LinearOpMode {


    private TrajectorySequence preloadSeqL, preloadSeqM, preloadSeqR, backdropSeq, parkSeq;
    public static double LPreload_x = 10, LPreload_y = 10, LPreload_heading = 10;
    public static double MPreload_x = 10, MPreload_y = 10, MPreload_heading = 10;
    public static double RPreload_x = 10, RPreload_y = 10, RPreload_heading = 10;
    public static double Backdrop_x = 10, Backdrop_y = 10, Backdrop_heading = 10;
    public static double Park_x = 10, Park_y = 10, Park_heading = 10;
    private final Pose2d preloadPosL = new Pose2d(LPreload_x,LPreload_y,LPreload_heading);
    private Pose2d preloadPosM = new Pose2d(MPreload_x,MPreload_y,MPreload_heading);
    private Pose2d preloadPosR = new Pose2d(RPreload_x,RPreload_y,RPreload_heading);
    private Pose2d backdropPos = new Pose2d(Backdrop_x,Backdrop_y,Backdrop_heading);
    private Pose2d parkPos = new Pose2d(Park_x,Park_y,Park_heading);

    private boolean updateMecanum = true, reachedBackdrop = false, retreatElevator = false, goingPreload = true;
    @Override
    public void runOpMode() throws InterruptedException {
        ControlHub ch = new ControlHub(hardwareMap);
        ExpansionHub eh = new ExpansionHub(hardwareMap);
        RedClose detection = new RedClose(hardwareMap,telemetry);
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

        waitForStart();
        switch (detection.getLocation()) {
            case MIDDLE:
                goingPreload = false;
                mecanumDrive.followTrajectorySequenceAsync(preloadSeqM);
                break;
            case RIGHT:
                goingPreload = false;
                mecanumDrive.followTrajectorySequenceAsync(preloadSeqR);
                break;
            default:
                goingPreload = false;
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
*/