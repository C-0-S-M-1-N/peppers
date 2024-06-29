package org.firstinspires.ftc.teamcode.Auto;

import android.os.Environment;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.OutTakeMTI;
import org.firstinspires.ftc.teamcode.detectionPipelines.BlueCloseDetectionPipeline;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.io.File;
import java.io.IOException;

@Config
@Autonomous(name = "BlueClose", preselectTeleOp = ".pipers \uD83C\uDF36", group = "auto")
public class BlueClose extends LinearOpMode {
    SampleMecanumDriveCancelable drive;
    OutTakeMTI outTake;
    Intake intake;

    public static Pose2d
            MiddlePurple = new Pose2d(24, -0.5, 0),
            MiddleYellow = new Pose2d(12.5, 31.5, Math.toRadians(60)),

            LeftPurple = new Pose2d(12, 7, Math.toRadians(357)),
            LeftYellow = new Pose2d(13.4, 28, Math.toRadians(76)),

            RightPurple = new Pose2d(15, -7, Math.toRadians(314)),
            RightYellow = new Pose2d(11, 40, Math.toRadians(57))
    ;

    public static Pose2d
            TrussToStack     = new Pose2d(4, -45, Math.PI/2.f),
            Stack            = new Pose2d(21, -76, Math.toRadians(110)),
            Stack2           = new Pose2d(34, -75, Math.toRadians(110)),
            BackBoardToTruss = new Pose2d(4, -9, Math.PI/2.f),
            Backdrop         = new Pose2d(12, 28, Math.toRadians(70)),
            TrussToStack_s     = new Pose2d(3, -45, Math.PI/2.f),
            BackBoardToTruss_s = new Pose2d(3, -9, Math.PI/2.f)

    ;
    int pixelsInStack = 5;
    int queue = 0;
    private boolean distanceSensorMesh = false;

    boolean ack = false;

    int intakeActive = 0;
    private boolean isInPreloadPhase = false, firstPathAfterPreload = true;
    private boolean pixelsUpdated = false, secondStack = false, wasATryLast = false, isAtStack = false;
    @Override
    public void runOpMode() throws InterruptedException {

        File file = new File(Environment.getExternalStorageDirectory(), OutTakeMTI.cacheFileName);
        secondStack = false;

        if(file.exists()){
            file.delete();
        }
        try {
            file.createNewFile();
        } catch (IOException e) {
            RobotLog.e("file not found");
        }

        drive = new SampleMecanumDriveCancelable(hardwareMap);

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        OpenCvCamera camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        BlueCloseDetectionPipeline detector = new BlueCloseDetectionPipeline(telemetry, true);

        camera.setPipeline(detector);
        // ------------------ OpenCv code
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                camera.startStreaming(432, 240, OpenCvCameraRotation.SENSOR_NATIVE);
            }

            @Override
            public void onError(int errorCode) {
                // ------------------ Tzeapa frate
            }

        });
        FtcDashboard.getInstance().startCameraStream(camera, 0);
//        OutTakeMTI.isInAutonomous = true;


        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub e = new ExpansionHub(hardwareMap, drive.getLocalizer());
        Controls cn = new Controls(gamepad1, gamepad2);
        ControlHub.telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        ExpansionHub.setInitialBackdropAngleRelativeToBot(90);
        Rev2mDistanceSensor distanceSensor = hardwareMap.get(Rev2mDistanceSensor.class, "redSensor");

        outTake = new OutTakeMTI();
        intake = new Intake();
        Intake.reversePower = -1;

        OutTakeMTI.timeToDrop = 0.2;
        OutTakeMTI.arm.rotationIndex = 0;



        TrajectorySequence left = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    outTake.setToPurplePlacing();
                    isInPreloadPhase = true;
                })
                .lineToLinearHeading(LeftPurple)
                .UNSTABLE_addDisplacementMarkerOffset(0.3, () -> {
                    Controls.DropRight = true;
                    Controls.DropRightAck = false;
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    OutTakeMTI.State.level = 1;
                    OutTakeMTI.elevator.setTargetPosition(3.5 * OutTakeMTI.STEP);
                    OutTakeMTI.align = true;
                    outTake.setToNormalPlacingFromPurplePixelPlacing();
                })
                .lineToLinearHeading(LeftYellow)
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    Controls.DropLeft = true;
                    Controls.DropLeftAck = false;
                })
                .addTemporalMarker(() -> {
                    OutTakeMTI.driverUpdated = true;
                })
                .addTemporalMarker(() -> {
                    isInPreloadPhase = false;
                    firstPathAfterPreload = true;
                    OutTakeMTI.arm.rotationIndex = 0;
                })
                .build();

        TrajectorySequence right = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    outTake.setToPurplePlacing();
                    isInPreloadPhase = true;
                })
                .lineToLinearHeading(RightPurple)
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropRightAck = false;
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    OutTakeMTI.State.level = 1;
                    OutTakeMTI.elevator.setTargetPosition(3.5 * OutTakeMTI.STEP);
                    OutTakeMTI.align = true;
                    outTake.setToNormalPlacingFromPurplePixelPlacing();
                })
                .lineToLinearHeading(RightYellow)
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    Controls.DropLeft = true;
                    Controls.DropLeftAck = false;
                })
                .addTemporalMarker(() -> {
                    OutTakeMTI.driverUpdated = true;
                })
                .addTemporalMarker(() -> {
                    isInPreloadPhase = false;
                    firstPathAfterPreload = true;
                    OutTakeMTI.arm.rotationIndex = 0;
                })
                .build();



        TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    outTake.setToPurplePlacing();
                    isInPreloadPhase = true;
                })
                .lineToLinearHeading(MiddlePurple)
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropRightAck = false;
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    OutTakeMTI.State.level = 1;
                    OutTakeMTI.elevator.setTargetPosition(3.5 * OutTakeMTI.STEP);
                    OutTakeMTI.align = true;
                    outTake.setToNormalPlacingFromPurplePixelPlacing();
                })
                .lineToLinearHeading(MiddleYellow)
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    Controls.DropLeft = true;
                    Controls.DropLeftAck = false;
                })
                .addTemporalMarker(() -> {
                    OutTakeMTI.driverUpdated = true;
                })
                .addTemporalMarker(() -> {
                    isInPreloadPhase = false;
                    firstPathAfterPreload = true;
                    OutTakeMTI.arm.rotationIndex = 0;
                })
                .build();

        ack = false;

        TrajectorySequence goToStackFromPreloadL = drive.trajectorySequenceBuilder(left.end())
                .setReversed(true)
                .addTemporalMarker(() -> {
                    firstPathAfterPreload = false;
                })
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(45))
                .splineToSplineHeading(BackBoardToTruss, Math.toRadians(-90))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(55, 5, DriveConstants.TRACK_WIDTH))
                .resetAccelConstraint()
                .splineToSplineHeading(TrussToStack, Math.toRadians(-90))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 5, DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(() -> {
                    intake.setPixelStackPosition(pixelsInStack);
                    intakeActive = 1;
                    pixelsUpdated = false;
                })
                .splineToSplineHeading(Stack, Math.toRadians(-45))
                .resetAccelConstraint()
                .addTemporalMarker(() -> {
                    isAtStack = true;
                })
                .build();

        TrajectorySequence goToStackFromPreloadR = drive.trajectorySequenceBuilder(right.end())
                .setReversed(true)
                .addTemporalMarker(() -> {
                    firstPathAfterPreload = false;
                })
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(48))
                .splineToSplineHeading(BackBoardToTruss, Math.toRadians(-90))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(60, 5, DriveConstants.TRACK_WIDTH))
                .resetAccelConstraint()
                .splineToSplineHeading(TrussToStack, Math.toRadians(-90))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 5, DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(() -> {
                    intake.setPixelStackPosition(pixelsInStack);
                    intakeActive = 1;
                    pixelsUpdated = false;
                })
                .splineToSplineHeading(Stack, Math.toRadians(-45))
                .resetAccelConstraint()
                .addTemporalMarker(() -> {
                    isAtStack = true;
                })
                .build();

        TrajectorySequence goToStackFromPreloadM = drive.trajectorySequenceBuilder(middle.end())
                .setReversed(true)
                .addTemporalMarker(() -> {
                    firstPathAfterPreload = false;
                })
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(48))
                .splineToSplineHeading(BackBoardToTruss, Math.toRadians(-90))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(55, 5, DriveConstants.TRACK_WIDTH))
                .resetAccelConstraint()
                .splineToSplineHeading(TrussToStack, Math.toRadians(-90))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(55, 5, DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(() -> {
                    intake.setPixelStackPosition(pixelsInStack);
                    intakeActive = 1;
                    pixelsUpdated = false;
                })
                .splineToSplineHeading(Stack, Math.toRadians(-45))
                .resetAccelConstraint()
                .addTemporalMarker(() -> {
                    isAtStack = true;
                })
                .build();



        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(Backdrop)
                .setReversed(true)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .splineToSplineHeading(BackBoardToTruss, Math.toRadians(-90))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(60, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(55))
                .splineToSplineHeading(TrussToStack, Math.toRadians(-90))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(55, 5, DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(() -> {
                    ack = false;
                })
                .addTemporalMarker(() -> {
                    intake.setPixelStackPosition(pixelsInStack);
                    intakeActive = 1;
                    pixelsUpdated = false;
                })
                .splineToSplineHeading(Stack, Math.toRadians(-45))
                .resetAccelConstraint()
                .addTemporalMarker(() -> {
                    isAtStack = true;
                })
                .build();



        TrajectorySequence takePixels = drive.trajectorySequenceBuilder(goToStack.end())
                .addTemporalMarker(() -> {
                    wasATryLast = false;
                    intakeActive = 1;
                    pixelsUpdated = false;
                    intake.setPixelStackPosition(pixelsInStack);
                })
                .forward(1)
                .back(1)
                .forward(1)
                .back(1)
                .addTemporalMarker(() -> {
                    intakeActive = -1;
                    if(!pixelsUpdated) pixelsInStack --;
                    if(pixelsInStack == 1) wasATryLast = true;
                })
                .build();

        TrajectorySequence takeFromSecondStack = drive.trajectorySequenceBuilder(takePixels.end())
                .addTemporalMarker(() -> {
                    intake.setPixelStackPosition(pixelsInStack);
                })
                .lineToLinearHeading(Stack2)
                .addTemporalMarker(() -> {
                    intakeActive = 1;
                    pixelsUpdated = false;
                    intake.setPixelStackPosition(pixelsInStack);
                })
                .forward(1)
                .back(1)
                .forward(1)
                .back(1)
                .addTemporalMarker(() -> {
                    intakeActive = -1;
                    if(!pixelsUpdated) pixelsInStack --;
                })
                .build();

        TrajectorySequence takeFromSecondStackWithoutLine = drive.trajectorySequenceBuilder(takeFromSecondStack.end())
                .addTemporalMarker(() -> {
                    intakeActive = 1;
                    pixelsUpdated = false;
                    intake.setPixelStackPosition(pixelsInStack);
                })
                .forward(1)
                .back(1)
                .forward(1)
                .back(1)
                .addTemporalMarker(() -> {
                    intakeActive = -1;
                    if(!pixelsUpdated) pixelsInStack --;
                })
                .build();

        TrajectorySequence goToBackDropFromSecondStack = drive.trajectorySequenceBuilder(takeFromSecondStack.end())
                .addTemporalMarker(() -> {
                    isAtStack = false;
                })
                .setReversed(false)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(55))
                .strafeLeft(1.2)
                .addTemporalMarker(() -> {
                    intakeActive = -1;
                })
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(55, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .splineToSplineHeading(TrussToStack_s, Math.toRadians(90))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(60, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(55))
                .splineToConstantHeading(new Vector2d(BackBoardToTruss_s.getX(), BackBoardToTruss_s.getY()), Math.toRadians(90))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(45, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(45))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    OutTakeMTI.State.level = 3;
                    Controls.ExtendElevator = true;
                    Controls.ExtendElevatorAck = false;
//                    distanceSensorMesh = true;
                })
                .addTemporalMarker(() -> {
                    intakeActive = 0;
                })
                .splineToSplineHeading(new Pose2d(Backdrop.getX(), Backdrop.getY() + 1, Backdrop.getHeading()), Math.toRadians(55))
                .UNSTABLE_addDisplacementMarkerOffset(-0.6, () -> {
                    OutTakeMTI.elevator.setLevel(6);
                })
                .waitSeconds(0.05)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                    Controls.DropLeftAck = false;
                    Controls.DropRightAck = false;
                })
                .waitSeconds(0.001)

                .build();

        TrajectorySequence goToBackDrop = drive.trajectorySequenceBuilder(takePixels.end())
                .addTemporalMarker(() -> {
                    intakeActive = -1;
                    isAtStack = false;
                })
                .setReversed(false)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 5, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(45))
                .splineToSplineHeading(TrussToStack_s, Math.toRadians(90))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(55))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(55, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(BackBoardToTruss_s.getX(), BackBoardToTruss_s.getY()), Math.toRadians(90))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(55, 4, DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(() -> {
                    intakeActive = 0;
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> {
                    OutTakeMTI.State.level = 1;
                    Controls.ExtendElevator = true;
                    Controls.ExtendElevatorAck = false;
//                    distanceSensorMesh = true;
                })
                .splineToSplineHeading(Backdrop, Math.toRadians(55))
                .UNSTABLE_addTemporalMarkerOffset(-0.5, () -> {
                    OutTakeMTI.elevator.setLevel(5);
                })
                .waitSeconds(0.05)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                    Controls.DropLeftAck = false;
                    Controls.DropRightAck = false;
                })

                .build();

        BlueCloseDetectionPipeline.Location location;

        while (opModeInInit()){
            location = detector.getLocation();
            if(location != null)
                telemetry.addData("case", location.toString());
            outTake.update();
            telemetry.update();
        }

        new Thread(() -> {
            camera.closeCameraDevice();
        }).start();

        location = detector.getLocation();

        switch (location){
            case LEFT:
                drive.followTrajectorySequenceAsync(left);
                break;
            case MIDDLE:
                drive.followTrajectorySequenceAsync(middle);
                break;
            case RIGHT:
                drive.followTrajectorySequenceAsync(right);
                break;
        }


        int order = 0;
        long time1 = System.currentTimeMillis();
        ElapsedTime autoTime = new ElapsedTime();

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
            if(order == 1 && (pixelsInStack == 0 || (!drive.isBusy() && pixelsInStack == 1)) && isAtStack){
                secondStack = true;
                pixelsInStack = 5;
                drive.breakFollowing();
                drive.followTrajectorySequenceAsync(takeFromSecondStack);
                pixelsUpdated = true;
                ack = true;
            } else if((order == 1 && OutTakeMTI.isFullOfPixels()) && isAtStack){
                pixelsInStack --;
                if(pixelsInStack < 0) pixelsInStack = 0;
                drive.breakFollowing();
                order ++;
                pixelsUpdated = true;
            }
            else if(order == 1 && OutTakeMTI.hasAPixel() && !ack && isAtStack){
                pixelsInStack --;
                drive.breakFollowing();
                ack = true;
                pixelsUpdated = true;
            }
            Controls.Intake = false;
            Controls.RevIntake = false;

            if(intakeActive == 1){
                Controls.Intake = true;
            } else if(intakeActive == -1){
                Controls.RevIntake = true;
                Intake.forceOut = true;
            }

            if(!isInPreloadPhase && !drive.isBusy()){
                switch (order) {
                    case 0:
                        if(autoTime.seconds() <= 25) {
                            if (firstPathAfterPreload) {
                                switch (location){
                                    case LEFT:
                                        drive.followTrajectorySequenceAsync(goToStackFromPreloadL);
                                        break;
                                    case MIDDLE:
                                        drive.followTrajectorySequenceAsync(goToStackFromPreloadM);
                                        break;
                                    case RIGHT:
                                        drive.followTrajectorySequenceAsync(goToStackFromPreloadR);
                                        break;
                                }
                            }
                            else drive.followTrajectorySequenceAsync(goToStack);
                            Pose2d pose = drive.getLocalizer().getPoseEstimate();
                            drive.setPoseEstimate(new Pose2d(pose.getX() - 0.4, pose.getY(), pose.getHeading()));
                            order++;
                        }
                        break;
                    case 1:
                        if(secondStack)
                            drive.followTrajectorySequenceAsync(takeFromSecondStackWithoutLine);
                        else drive.followTrajectorySequenceAsync(takePixels);
                        break;
                    case 2:
//                        if(autoTime.seconds() < 15) break;
                        if(secondStack)
                            drive.followTrajectorySequenceAsync(goToBackDropFromSecondStack);
                        else drive.followTrajectorySequenceAsync(goToBackDrop);
                        order = 0;
                    default:
                        break;
                }
            }

            if(distanceSensorMesh){
                Pose2d robotPos = drive.getPoseEstimate();
                double distance = distanceSensor.getDistance(DistanceUnit.INCH);
                robotPos = new Pose2d(distance, robotPos.getY(), robotPos.getHeading());
//                drive.setPoseEstimate(robotPos);
                distanceSensorMesh = false;
            }
            drive.update();

            intake.update_values();
            intake.update();
            outTake.update();
            cn.loop();
            ControlHub.telemetry.addData("pixels in stack", pixelsInStack);
            ControlHub.telemetry.addData("Time Left", 30 - autoTime.seconds());
            ControlHub.telemetry.update();
            if(30 - autoTime.seconds() <= -0.1) stop();
        }
        OutTakeMTI.timeToDrop = 0.3;
        Intake.reversePower = -1;
        OutTakeMTI.isInAutonomous = false;
    }
}