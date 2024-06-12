package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.OutTakeMTI;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.internals.AprilTagDetector;
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
    MiddleYellow = new Pose2d(15, -29, Math.toRadians(302));

    public static Pose2d
    TrussToStack     = new Pose2d(8, 45, -Math.PI/2.f),
    Stack            = new Pose2d(26, 72, Math.toRadians(250)),
    BackBoardToTruss = new Pose2d(8, -1, -Math.PI/2.f),
    Backdrop         = new Pose2d(17, -28, Math.toRadians(248));
    int pixelsInStack = 5;
    int queue = 0;
    private boolean distanceSensorMesh = false;

    boolean ack = false;

    int intakeActive = 0;
    private boolean isInPreloadPhase = false, firstPathAfterPreload = true;
    private boolean isTakingPixels = false;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDriveCancelable(hardwareMap);

        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub e = new ExpansionHub(hardwareMap, drive.getLocalizer());
        Controls cn = new Controls(gamepad1, gamepad2);
        ControlHub.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ExpansionHub.setInitialBackdropAngleRelativeToBot(-90);
        Rev2mDistanceSensor distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "redSensor");

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
                .waitSeconds(0.07)
                .addTemporalMarker(() -> {
                    OutTakeMTI.State.level = 1;
                    OutTakeMTI.elevator.setTargetPosition(2.5 * OutTakeMTI.STEP);
                    OutTakeMTI.align = true;
                    outTake.setToNormalPlacingFromPurplePixelPlacing();
                })
                .lineToLinearHeading(MiddleYellow)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropRightAck = false;
                })
                .addTemporalMarker(() -> {
                    OutTakeMTI.driverUpdated = true;
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    isInPreloadPhase = false;
                    firstPathAfterPreload = true;
                })
                .build();

        TrajectorySequence preload = middle;

        TrajectorySequence goToStackFromPreload = drive.trajectorySequenceBuilder(preload.end())
                .setReversed(true)
                .addTemporalMarker(() -> {
                    firstPathAfterPreload = false;
                })
                .splineToSplineHeading(BackBoardToTruss, Math.toRadians(90))
                .splineToSplineHeading(TrussToStack, Math.toRadians(90))
                .splineToSplineHeading(Stack, Math.toRadians(45))
                .build();

        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(Backdrop)
                .setReversed(true)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(55, 5, DriveConstants.TRACK_WIDTH))
                .splineToSplineHeading(BackBoardToTruss, Math.toRadians(90))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 5, DriveConstants.TRACK_WIDTH))
                .splineToSplineHeading(TrussToStack, Math.toRadians(90))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(65, 5, DriveConstants.TRACK_WIDTH))
                .splineToSplineHeading(Stack, Math.toRadians(45))
                .build();

        ElapsedTime intakeReveresTime = new ElapsedTime();

        TrajectorySequence takePixels = drive.trajectorySequenceBuilder(goToStack.end())
                .addTemporalMarker(() -> {
                    ack = false;
                    intake.setPixelStackPosition(pixelsInStack);
                    intakeActive = 1;
                })
                .back(2)
                .forward(2)
                .back(2)
                .forward(2)
                .addTemporalMarker(() -> {
                    intakeActive = -1;
                    intakeReveresTime.reset();
                })
                .waitSeconds(0.1)
                .build();
        TrajectorySequence goToBackDrop = drive.trajectorySequenceBuilder(takePixels.end())
                .setReversed(false)
                .splineToSplineHeading(TrussToStack, Math.toRadians(-90))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(65, 5, DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(() -> {
                    intakeActive = 0;
//                    distanceSensorMesh = true;
                })
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(55, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(BackBoardToTruss.getX(), BackBoardToTruss.getY()), Math.toRadians(-90))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(70, 5, DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(() -> {
                    OutTakeMTI.State.level = 5;
                    Controls.ExtendElevator = true;
                    Controls.ExtendElevatorAck = false;
//                    distanceSensorMesh = true;
                })
                .waitSeconds(0.1)
                .splineToSplineHeading(Backdrop, Math.toRadians(-45))
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                    Controls.DropLeftAck = false;
                    Controls.DropRightAck = false;
                })
                .waitSeconds(0.1)

                .build();

        drive.followTrajectorySequenceAsync(preload);

        while (opModeInInit()){
            outTake.update();
        }

        int order = 0;
        long time1 = System.currentTimeMillis();

        while(opModeIsActive()){
            ControlHub.ControlHubModule.clearBulkCache();
            ExpansionHub.ExpansionHubModule.clearBulkCache();
//            e.update(false, drive.getLocalizer().getPoseEstimate().getX(), drive.getLocalizer().getPoseEstimate().getY());
            ExpansionHub.ImuYawAngle = Math.toDegrees(drive.getPoseEstimate().getHeading()) - ExpansionHub.beforeReset;
            if((System.currentTimeMillis() - time1) >= 1.0 / 4){
                double Yawn = ExpansionHub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Yawn));
//                ExpansionHub.ImuYawAngle = Yawn - ExpansionHub.beforeReset;
                time1 = System.currentTimeMillis();
            }
            if(order == 1 && OutTakeMTI.isFullOfPixels()){
                pixelsInStack --;
                drive.breakFollowing();
                order ++;
            }
            if(order == 1 && OutTakeMTI.hasAPixel() && !ack){
                pixelsInStack --;
                drive.breakFollowing();
                ack = true;
            }
            Controls.Intake = false;
            Controls.RevIntake = false;

            if(intakeActive == 1){
                Controls.Intake = true;
            } else if(intakeActive == -1 && intakeReveresTime.seconds() <= 0.1){
                Controls.RevIntake = true;
            }

            if(!isInPreloadPhase && !drive.isBusy()){
                switch (order){
                    case 0:
                        if(firstPathAfterPreload) drive.followTrajectorySequenceAsync(goToStackFromPreload);
                        else drive.followTrajectorySequenceAsync(goToStack);
                        order ++;
                        break;
                    case 1:
                        drive.followTrajectorySequenceAsync(takePixels);
                        break;
                    case 2:
                        drive.followTrajectorySequenceAsync(goToBackDrop);
                        order = 0;
                    default:
                        break;
                }
            }

            if(distanceSensorMesh){
                Pose2d robotPos = drive.getPoseEstimate();
                double distance = distanceSensor.getDistance(DistanceUnit.INCH);
                robotPos = new Pose2d(distance, robotPos.getY(), robotPos.getHeading());
                drive.setPoseEstimate(robotPos);
                distanceSensorMesh = false;
            }
            drive.update();

            intake.update_values();
            intake.update();
            outTake.update();
            cn.loop();
            ControlHub.telemetry.update();
        }
    }
}