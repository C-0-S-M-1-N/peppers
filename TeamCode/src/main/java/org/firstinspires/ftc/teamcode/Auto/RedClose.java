package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.OutTake;
import org.firstinspires.ftc.teamcode.Parts.OutTakeMTI;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Config
@Autonomous(name = "RedClose", preselectTeleOp = ".pipers \uD83C\uDF36")
public class RedClose extends LinearOpMode {
    SampleMecanumDriveCancelable drive;
    OutTakeMTI outTake;
    Intake intake;

    public static Pose2d
    MiddlePurple = new Pose2d(25, -1, 0),
    MiddleYellow = new Pose2d(30, -24, -Math.PI / 2.f);

    public static Pose2d
    UnderTruss = new Pose2d(0, 0, 0),
    PreStack   = new Pose2d(0, 0, 0),
    Stack      = new Pose2d(0, 0, 0),
    BackBoardToTruss = new Pose2d(0, 0, 0),
    StackToTruss     = new Pose2d(0, 0, 0);

    int pixelsInStack = 5;
    int queue = 0;


    int intakeActive = 0;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDriveCancelable(hardwareMap);

        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub e = new ExpansionHub(hardwareMap, drive.getLocalizer());
        Controls cn = new Controls(gamepad1, gamepad2);
        ControlHub.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        outTake = new OutTakeMTI();
        intake = new Intake();


        TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    outTake.setToPurplePlacing();
                })
                .lineToLinearHeading(MiddlePurple)
                .addTemporalMarker(() -> {
                    Controls.DropLeft = true;
                    Controls.DropLeftAck = false;
                })
                .addTemporalMarker(() -> {
                    OutTakeMTI.State.level = 1;
                    OutTakeMTI.elevator.setTargetPosition(2 * OutTakeMTI.STEP);
                    outTake.setToNormalPlacingFromPurplePixelPlacing();
                })
                .lineToLinearHeading(MiddleYellow)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropRightAck = false;
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    OutTakeMTI.elevator.setTargetPosition(5 * OutTakeMTI.STEP);
                })
                .addTemporalMarker(() -> {
                    Controls.RetractElevator = true;
                    Controls.RetractElevatorAck = false;
                })
                .build();

        drive.followTrajectorySequenceAsync(middle);

        while (opModeInInit()){
            outTake.update();
        }

//        waitForStart(); // TODO: add save&load servo position data reading

        while(opModeIsActive()){
            ControlHub.ControlHubModule.clearBulkCache();
            ExpansionHub.ExpansionHubModule.clearBulkCache();
            /*if(intakeActive == 1){
               Controls.Intake = true;
            } else if(intakeActive == -1){
                Controls.RevIntake = true;
            }
            if(outTake.gotAPixel()){
                pixelsInStack --;
            }
            if(!drive.isBusy()){
                queue ++;
                if(queue == 1) drive.followTrajectorySequenceAsync(preloadToStack);
                else if(queue == 2) drive.followTrajectorySequenceAsync(takeFromStack);
            }*/

            intake.update_values();
            intake.update();
            drive.update();
            outTake.update();
            cn.loop();
        }



    }
}
