package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.internals.AprilTagDetector.cx;
import static org.firstinspires.ftc.teamcode.internals.AprilTagDetector.cy;
import static org.firstinspires.ftc.teamcode.internals.AprilTagDetector.fx;
import static org.firstinspires.ftc.teamcode.internals.AprilTagDetector.fy;
import static org.firstinspires.ftc.teamcode.internals.AprilTagDetector.tagSize;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Parts.OutTake;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.internals.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utils.AutoServo;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(group = "Autos", preselectTeleOp = "pipers \\uD83C\\uDF36", name = "NotShitRedClose")
@Config
public class NotShitRedClose extends LinearOpMode {
    private org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable mecanumDrive;
    RedCloseTrajectory t;
    OpenCvCamera camera;

    final double METERS_TO_INCHES = 39.37;
    public static double TARGET_ID = 5;
    public static double OFFSET_X = 21.2;
    public static double OFFSET_Y = 37.5;
    AprilTagDetectionPipeline pipeline;

    int caz;
    public static double init_x = 0 , init_y = 0, init_heading = 0;
    public static double LPreload_x = 33, LPreload_y = -15, LPreload_heading = -90;
    public static double MPreload_x = 39, MPreload_y = -4, MPreload_heading = -90;
    public static double RPreload_x = 32, RPreload_y = 7, RPreload_heading = -90;

    public static double LBackdrop_x = 32, LBackdrop_y = -36, LBackdrop_heading = -90;
    public static double MBackdrop_x = 26, MBackdrop_y = -36, MBackdrop_heading = -90;
    public static double RBackdrop_x = 20, RBackdrop_y = -36, RBackdrop_heading = -90;
    public static double Park_x = 0, Park_y = -34, Park_heading = -91;
    public static double Transit_x = 11, Transit_y = 2, Transit_heading = -0.1;
    private final Pose2d preloadPosL = new Pose2d(LPreload_x,LPreload_y,Math.toRadians(LPreload_heading));
    private Pose2d preloadPosM = new Pose2d(MPreload_x,MPreload_y,Math.toRadians(MPreload_heading));
    private Pose2d preloadPosR = new Pose2d(RPreload_x,RPreload_y,Math.toRadians(RPreload_heading));
    private Pose2d initPos = new Pose2d(init_x, init_y, Math.toRadians(init_heading));

    private Pose2d LbackdropPos = new Pose2d(LBackdrop_x,LBackdrop_y,Math.toRadians(LBackdrop_heading));
    private Pose2d MbackdropPos = new Pose2d(MBackdrop_x,MBackdrop_y,Math.toRadians(MBackdrop_heading));
    private Pose2d RbackdropPos = new Pose2d(RBackdrop_x,RBackdrop_y,Math.toRadians(RBackdrop_heading));
    private Pose2d parkPos = new Pose2d(Park_x,Park_y,Math.toRadians(Park_heading));
    private Pose2d transitPos = new Pose2d(Transit_x, Transit_y, Math.toRadians(Transit_heading));
    private Pose2d prePreTruss = new Pose2d(2, -14, Math.toRadians(-90));
    private Pose2d preTruss = new Pose2d(2, -0, Math.toRadians(-90));
    private Pose2d preStack = new Pose2d(2, 70, Math.toRadians(-90));
    private Pose2d prePreStack = new Pose2d(2, 60, Math.toRadians(-90));

    private Pose2d stack = new Pose2d(28.5, 68, Math.toRadians(-90));
    private Controls c;
    private AutoServo intakeServo;

    private NanoClock clock;
    private double intakeStart = 0;
    private boolean TO_INTAKE = false;
    private double startTime = 0;
    private boolean REVERSE = false;
    private boolean TO_RELOCALIZE = false;

    private boolean PRELOAD_PLACED = false;

    private double angle = 75;
    private double step = 4;
    private int level = 2;
    private double startDetection = 0;

    public Pose2d STACK;

    @SuppressLint("SuspiciousIndentation")
    public void runOpMode() throws InterruptedException{
        c = new Controls(gamepad1, gamepad2);
        ControlHub ch = new ControlHub(hardwareMap);
        ExpansionHub eh = new ExpansionHub(hardwareMap);
        clock = NanoClock.system();
        mecanumDrive = new SampleMecanumDriveCancelable(hardwareMap);
        t = new RedCloseTrajectory(mecanumDrive);
        intakeServo = new AutoServo(SERVO_PORTS.S3, true, false, 0, AutoServo.type.AXON);
        intakeServo.setPosition(90);

        OutTake outTake = new OutTake(hardwareMap, telemetry);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        OutTake.useControls = false;

        // ------------------ OpenCv initialisation code
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        ObjectDetectionPipelineRED detector = new ObjectDetectionPipelineRED(telemetry, false);

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

        TrajectorySequence preloadRight = mecanumDrive.trajectorySequenceBuilder(initPos)
                .lineToLinearHeading(preloadPosL)
                .addTemporalMarker(() -> {
                    pipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);
                    camera.setPipeline(pipeline);
                    camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
                    {
                        @Override
                        public void onOpened()
                        {
                            //TODO ADJUST CAMERA
                            camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
                        }

                        @Override
                        public void onError(int errorCode)
                        {

                        }
                    });
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    OutTake.STATES.currentLevel = 2;
                    Controls.ExtendElevator = true;
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                })
                .forward(4)
                .addTemporalMarker(() -> {
                    TO_RELOCALIZE = true;
                })
                .build();
        TrajectorySequence preloadMiddle = mecanumDrive.trajectorySequenceBuilder(initPos)
                .lineToLinearHeading(preloadPosM)
                .addTemporalMarker(() -> {
                    pipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);
                    camera.setPipeline(pipeline);
                    camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
                    {
                        @Override
                        public void onOpened()
                        {
                            //TODO ADJUST CAMERA
                            camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
                        }

                        @Override
                        public void onError(int errorCode)
                        {

                        }
                    });
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    OutTake.STATES.currentLevel = 2;
                    Controls.ExtendElevator = true;
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                })
                .forward(4)
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    TO_RELOCALIZE = true;
                })
                .build();
        TrajectorySequence preloadLeft = mecanumDrive.trajectorySequenceBuilder(initPos)
                .lineToLinearHeading(preloadPosR)
                .addTemporalMarker(() -> {
                    pipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);
                    camera.setPipeline(pipeline);
                    camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
                    {
                        @Override
                        public void onOpened()
                        {
                            //TODO ADJUST CAMERA
                            camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
                        }

                        @Override
                        public void onError(int errorCode)
                        {

                        }
                    });
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    OutTake.STATES.currentLevel = 2;
                    Controls.ExtendElevator = true;
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                })
                .forward(4)
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    TO_RELOCALIZE = true;
                })
                .build();

        TrajectorySequence bipBupL = mecanumDrive.trajectorySequenceBuilder(LbackdropPos)
                .lineToLinearHeading(prePreTruss)
                .splineToConstantHeading(new Vector2d(preTruss.getX(), preTruss.getY()), -preTruss.getHeading())
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 2.54, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(prePreStack.getX(), prePreStack.getY()), -prePreStack.getHeading())
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(35, 2.54, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(stack.getX(), stack.getY()), -stack.getHeading())
                .waitSeconds(0.01)
                .back(5)
                .addTemporalMarker(() -> {
                    TO_INTAKE = true;
                    STACK = new Pose2d(stack.getX(), stack.getY() + 5, stack.getHeading());
                })
                .build();

        TrajectorySequence bipBupR = mecanumDrive.trajectorySequenceBuilder(RbackdropPos)
                .lineToLinearHeading(prePreTruss)
                .splineToConstantHeading(new Vector2d(preTruss.getX(), preTruss.getY()), -preTruss.getHeading())
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 2.54, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(prePreStack.getX(), prePreStack.getY()), -prePreStack.getHeading())
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(35, 2.54, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(stack.getX(), stack.getY()), -stack.getHeading())
                .waitSeconds(0.01)
                .back(5)
                .addTemporalMarker(() -> {
                    TO_INTAKE = true;
                    STACK = new Pose2d(stack.getX(), stack.getY() + 5, stack.getHeading());
                })
                .build();

        TrajectorySequence bipBupM = mecanumDrive.trajectorySequenceBuilder(MbackdropPos)
                .lineToLinearHeading(prePreTruss)
                .splineToConstantHeading(new Vector2d(preTruss.getX(), preTruss.getY()), -preTruss.getHeading())
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 2.54, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(prePreStack.getX(), prePreStack.getY()), -prePreStack.getHeading())
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(35, 2.54, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(stack.getX(), stack.getY()), -stack.getHeading())
                .waitSeconds(0.01)
                .back(5)
                .addTemporalMarker(() -> {
                    TO_INTAKE = true;
                    STACK = new Pose2d(stack.getX(), stack.getY() + 5, stack.getHeading());
                })
                .build();

        TrajectorySequence yellow_L = mecanumDrive.trajectorySequenceBuilder(preloadLeft.end())
                .lineToLinearHeading(LbackdropPos)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                })
                .waitSeconds(0.15)
                .addTemporalMarker(() -> {
                    Controls.RetractElevator = true;
                    if(clock.seconds() - startTime <= 19) {
                        mecanumDrive.followTrajectorySequenceAsync(bipBupL);
                    } else {
                        mecanumDrive.followTrajectorySequenceAsync(mecanumDrive.trajectorySequenceBuilder(LbackdropPos).lineToLinearHeading(parkPos).build());
                    }
                })
                .build();

        TrajectorySequence yellow_R = mecanumDrive.trajectorySequenceBuilder(preloadRight.end())
                .lineToLinearHeading(RbackdropPos)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                })
                .waitSeconds(0.15)
                .addTemporalMarker(() -> {
                    Controls.RetractElevator = true;
                    if(clock.seconds() - startTime <= 19) {
                        mecanumDrive.followTrajectorySequenceAsync(bipBupR);
                    } else {
                        mecanumDrive.followTrajectorySequenceAsync(mecanumDrive.trajectorySequenceBuilder(RbackdropPos).lineToLinearHeading(parkPos).build());
                    }
                })
                .build();

        TrajectorySequence yellow_M = mecanumDrive.trajectorySequenceBuilder(preloadMiddle.end())
                .lineToLinearHeading(MbackdropPos)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                })
                .waitSeconds(0.15)
                .addTemporalMarker(() -> {
                    Controls.RetractElevator = true;
                    if(clock.seconds() - startTime <= 19) {
                        mecanumDrive.followTrajectorySequenceAsync(bipBupM);
                    } else {
                        mecanumDrive.followTrajectorySequenceAsync(mecanumDrive.trajectorySequenceBuilder(MbackdropPos).lineToLinearHeading(parkPos).build());
                    }
                })
                .build();

        TrajectorySequence fromBackboardToStack = mecanumDrive.trajectorySequenceBuilder(new Pose2d(MBackdrop_x, MBackdrop_y + 12, MBackdrop_heading))
                .lineToLinearHeading(MbackdropPos)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.15, () -> {
                    Controls.RetractElevator = true;
                })
                .lineToLinearHeading(parkPos)
                .build();

        TrajectorySequence fromStackToBackboard = mecanumDrive.trajectorySequenceBuilder(bipBupL.end())
                .waitSeconds(0.5)
                .addTemporalMarker(() -> {
                    intakeServo.setAngle(90);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0.65);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                })
                .lineToLinearHeading(prePreStack)
                .lineToLinearHeading(preTruss)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(60, 2.54, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(prePreTruss.getX(), prePreTruss.getY()), prePreTruss.getHeading())
                .addTemporalMarker(() -> {
                    OutTake.STATES.currentLevel = 3;
                    Controls.ExtendElevator = true;
                })
                .splineToConstantHeading(new Vector2d(MBackdrop_x, MBackdrop_y + 12), MBackdrop_heading)
                .UNSTABLE_addTemporalMarkerOffset(-2, () -> {
                    pipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);
                    camera.setPipeline(pipeline);
                    camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
                    {
                        @Override
                        public void onOpened()
                        {
                            //TODO ADJUST CAMERA
                            camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
                        }

                        @Override
                        public void onError(int errorCode)
                        {

                        }
                    });
                })
                .addTemporalMarker(() -> {
                    TO_RELOCALIZE = true;
                })
                .build();

        waitForStart();

        startTime = clock.seconds();

        switch (detector.getLocation()) {
            case LEFT:
                telemetry.addLine("LEFT CASE");
                caz = 1;
                mecanumDrive.followTrajectorySequenceAsync(preloadLeft);
                break;
            case MIDDLE:
                telemetry.addLine("MIDDLE CASE");
                caz = 2;
                mecanumDrive.followTrajectorySequenceAsync(preloadMiddle);

                break;
            case RIGHT:
                telemetry.addLine("RIGHT CASE");
                caz = 3;
                mecanumDrive.followTrajectorySequenceAsync(preloadRight);
                break;
            default:
                // not used
                telemetry.addData("location" , "nu stiu boss");
                mecanumDrive.followTrajectorySequenceAsync(preloadRight);
                break;
        }

        new Thread(() -> {
            camera.stopStreaming();
        }).start();

        while(opModeIsActive() && !isStopRequested()){
            mecanumDrive.update();
            outTake.update();
            outTake.runTelemetry();
            c.loop();
            intakeServo.update();

            if(TO_INTAKE) {
                if(intakeStart == 0) {
                    intakeStart = clock.seconds();
                }

                if(REVERSE) {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0.65);
                    intakeServo.setAngle(90);
                } else {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, -1);
                    intakeServo.setAngle(angle);
                }

                if((clock.seconds() - intakeStart > 1.5 && !REVERSE) || (clock.seconds() - intakeStart > 0.5 && REVERSE)) {
                    REVERSE = !REVERSE;
                    intakeStart = clock.seconds();
                    angle -= step;
                    if(angle < 40) angle = 40;
                }
                if(OutTake.fullPixel() || clock.seconds() - startTime > 21) {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, -1);
                    TO_INTAKE = false;
                    mecanumDrive.followTrajectorySequenceAsync(fromStackToBackboard);
                    intakeStart = 0;
                } else if(!mecanumDrive.isBusy()) {
                    mecanumDrive.followTrajectorySequenceAsync(chipiChapa());
                }

            }

            if(TO_RELOCALIZE) {
                if(startDetection == 0) startDetection = clock.seconds();

                ArrayList<AprilTagDetection> detections = pipeline.getLatestDetections();
                telemetry.addData("detected Apriltags: ", detections.size());

                double offset_x = 0;
                AprilTagDetection target_tag = null;
                for (AprilTagDetection detected : detections) {
                    if(detected.id == TARGET_ID) {
                        target_tag = detected;

                        telemetry.addData("id: ", detected.id);
                        telemetry.addData("x: ", detected.center.x * METERS_TO_INCHES); // this is X in global pose
                        telemetry.addData("y: ", detected.center.y * METERS_TO_INCHES);
                        telemetry.addData("z: ", detected.pose.z * METERS_TO_INCHES); // this is Y in global pose
                        break;

                    } else if(detected.id == TARGET_ID + 1) {
                        target_tag = detected;

                        telemetry.addData("id: ", detected.id);
                        telemetry.addData("x: ", detected.center.x * METERS_TO_INCHES); // this is X in global pose
                        telemetry.addData("y: ", detected.center.y * METERS_TO_INCHES);
                        telemetry.addData("z: ", detected.pose.z * METERS_TO_INCHES); // this is Y in global pose
                        offset_x = -6;
                    } else if(detected.id == TARGET_ID - 1) {
                        target_tag = detected;

                        telemetry.addData("id: ", detected.id);
                        telemetry.addData("x: ", detected.center.x * METERS_TO_INCHES); // this is X in global pose
                        telemetry.addData("y: ", detected.center.y * METERS_TO_INCHES);
                        telemetry.addData("z: ", detected.pose.z * METERS_TO_INCHES); // this is Y in global pose
                        offset_x = 6;
                    }

                }

                if(target_tag != null) {
                    telemetry.addData("pose_from_tag_x: ", OFFSET_X + target_tag.pose.x * METERS_TO_INCHES + offset_x);
                    telemetry.addData("pose_from_tag_y: ", -OFFSET_Y + target_tag.pose.z * METERS_TO_INCHES);

                    Pose2d pose = mecanumDrive.getPoseEstimate();
                    telemetry.addData("pose_from_odo_x: ", pose.getX());
                    telemetry.addData("pose_from_odo_y: ", pose.getY());
                    mecanumDrive.setPoseEstimate(new Pose2d(OFFSET_X + target_tag.pose.x * METERS_TO_INCHES + offset_x, -OFFSET_Y + target_tag.pose.z * METERS_TO_INCHES, pose.getHeading()));

                    TO_RELOCALIZE = false;
                    new Thread(() -> {
                        camera.closeCameraDevice();
                    }).start();

                    if(caz == 1 && !PRELOAD_PLACED) mecanumDrive.followTrajectorySequenceAsync(yellow_L);
                    if(caz == 2 && !PRELOAD_PLACED) mecanumDrive.followTrajectorySequenceAsync(yellow_M);
                    if(caz == 3 && !PRELOAD_PLACED) mecanumDrive.followTrajectorySequenceAsync(yellow_R);
                    else mecanumDrive.followTrajectorySequenceAsync(fromBackboardToStack);
                    PRELOAD_PLACED = true;
                }
            }

            telemetry.update();
        }
    }
    public TrajectorySequence chipiChapa() {
        return mecanumDrive.trajectorySequenceBuilder(new Pose2d(STACK.getX(), STACK.getY(), STACK.getHeading())).forward(3).back(3).build();
    }
}