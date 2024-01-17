package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.OutTake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(group = "Autos", preselectTeleOp = "pipers \\uD83C\\uDF36", name = "AutoBlueClose")
public class AutoBlueClose extends LinearOpMode {
    private BeautifulLocation objDetection;
    private org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive mecanumDrive;

    private TrajectorySequence SequenceToGetPixel, SequenceToBackdrop;
    private Pose2d placePixel, lineToStack, toBackdrop;
    private Pose2d toStack, park1, park2;
    private boolean updateMecanum = true, reachedBackdrop = false;

    @Override
    public void runOpMode() throws InterruptedException{
        Controls c = new Controls(gamepad1, gamepad2);
        mecanumDrive = new SampleMecanumDrive(hardwareMap);

        OutTake outTake = new OutTake(hardwareMap, telemetry);
        Intake intake = new Intake();

        objDetection = new BeautifulLocation(BeautifulLocation.Team.BLUE);

        SequenceToGetPixel = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(placePixel)
                .lineToLinearHeading(lineToStack)
                .lineToLinearHeading(toStack)
                .build();

        SequenceToBackdrop = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(toBackdrop)
                .lineToLinearHeading(park1)
                .lineToLinearHeading(park2)
                .build();

        mecanumDrive.followTrajectorySequenceAsync(SequenceToGetPixel);

        waitForStart();

        BeautifulLocation.Case caseDetected = objDetection.getCase();

        while(opModeIsActive() && !isStopRequested()){
            if(!mecanumDrive.isBusy()){
                Controls.Intake = true;
                updateMecanum = false;
                if(outTake.fullPixel()){
                    mecanumDrive.followTrajectorySequenceAsync(SequenceToBackdrop);
                    updateMecanum = true;
                }
            }
            if(mecanumDrive.getPoseEstimate() == toBackdrop){
                if(!reachedBackdrop){
                    OutTake.STATES.currentLevel = 2;
                    Controls.ExtendElevator = true;
                }
                updateMecanum = false;
                if(outTake.STATE == OutTake.STATES.IDLE && reachedBackdrop){
                    Controls.DropLeft = true;
                    Controls.DropRight = true;
                    updateMecanum = true;
                }
                reachedBackdrop = true;
            }
            if(updateMecanum)
                mecanumDrive.update();
            intake.update();
            outTake.update();
            c.loop();
        }

    }
}
