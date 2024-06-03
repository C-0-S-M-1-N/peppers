package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

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
    TrussToStack   = new Pose2d(6, 51, -Math.PI/2.f),
    Stack      = new Pose2d(25, 78, Math.toRadians(250)),
    BackBoardToTruss = new Pose2d(4.5, -1, -Math.PI/2.f),
    Backdrop         = new Pose2d(12, -28, Math.toRadians(302));
    int pixelsInStack = 5;
    int queue = 0;


    int intakeActive = 0;
    private boolean isInPreloadPhase = false;
    private boolean isTakingPixels = false;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDriveCancelable(hardwareMap);

        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub e = new ExpansionHub(hardwareMap, drive.getLocalizer());
        Controls cn = new Controls(gamepad1, gamepad2);
        ControlHub.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ExpansionHub.setInitialBackdropAngleRelativeToBot(-90);

        outTake = new OutTakeMTI();
        intake = new Intake();


        TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    outTake.setToPurplePlacing();
                    isInPreloadPhase = true;
                })
                .lineToLinearHeading(MiddlePurple)
                .addTemporalMarker(() -> {
                    Controls.DropLeft = true;
                    Controls.DropLeftAck = false;
                })
                .addTemporalMarker(() -> {
                    OutTakeMTI.State.level = 1;
                    OutTakeMTI.elevator.setTargetPosition(2 * OutTakeMTI.STEP);
                    OutTakeMTI.align = true;
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
                    isInPreloadPhase = false;
                })
                .waitSeconds(0.8)
                .lineToLinearHeading(BackBoardToTruss)
                .build();

        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(BackBoardToTruss)
                .lineToLinearHeading(TrussToStack)
                .lineToLinearHeading(Stack)
                .build();

        ElapsedTime intakeReveresTime = new ElapsedTime();

        TrajectorySequence takePixels = drive.trajectorySequenceBuilder(goToStack.end())
                .addTemporalMarker(() -> {
                    intakeActive = 1;
                })
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    intakeActive = -1;
                    intakeReveresTime.reset();
                })
                .build();
        TrajectorySequence goToBackDrop = drive.trajectorySequenceBuilder(takePixels.end())
                .lineToLinearHeading(TrussToStack)
                .addTemporalMarker(() -> {
                    intakeActive = 0;
                })
                .lineToLinearHeading(BackBoardToTruss)
                .addTemporalMarker(() -> {
                    OutTakeMTI.State.level = 3;
                    Controls.ExtendElevator = true;
                    Controls.ExtendElevatorAck = false;
                })
                .lineToLinearHeading(Backdrop)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                    Controls.DropLeftAck = false;
                    Controls.DropRightAck = false;
                })
                .waitSeconds(0.1)
                .build();

        drive.followTrajectorySequenceAsync(middle);

        while (opModeInInit()){
            outTake.update();
        }

        int order = 0;

        while(opModeIsActive()){
            ControlHub.ControlHubModule.clearBulkCache();
            ExpansionHub.ExpansionHubModule.clearBulkCache();
            e.update(false, drive.getLocalizer().getPoseEstimate().getX(), drive.getLocalizer().getPoseEstimate().getY());
            if(order == 1 && OutTakeMTI.isFullOfPixels()){
                drive.breakFollowing();
                order ++;
            }
            Controls.Intake = false;
            Controls.RevIntake = false;

            if(intakeActive == 1){
                Controls.Intake = true;
            }
            if(intakeActive == -1 && intakeReveresTime.seconds() <= 1){
                Controls.RevIntake = true;
            }

            if(!isInPreloadPhase && !drive.isBusy()){
                switch (order){
                    case 0:
                        drive.followTrajectorySequenceAsync(goToStack);
                        order ++;
                        break;
                    case 1:
                        drive.followTrajectorySequenceAsync(takePixels);
                        order ++; // TODO: delete this line dupa ce face ipate clestii sa mearga
                        break;
                    case 2:
                        drive.followTrajectorySequenceAsync(goToBackDrop);
                        order = 0;
                    default:
                        break;
                }
            }

            intake.update_values();
            intake.update();
            drive.update();
            outTake.update();
            cn.loop();
            ControlHub.telemetry.update();
        }
    }
}
