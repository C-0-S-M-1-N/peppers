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
import org.firstinspires.ftc.teamcode.internals.AprilTagDetector;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utils.AutoServo;
import org.opencv.core.Mat;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Autonomous(group = "Autos", preselectTeleOp = "pipers \\uD83C\\uDF36", name = "NotShitRedFar")
@Config
public class NotShitRedFar extends LinearOpMode {
    private org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable mecanumDrive;
    RedCloseTrajectory t;
    OpenCvCamera camera;
    final double METERS_TO_INCHES = 39.37;
    public static double TARGET_ID = 5;
    public static double OFFSET_X = 30;
    public static double OFFSET_Y = 79.5 - 23 + 39;
    AprilTagDetectionPipeline pipeline;

    int caz;
    public static double init_x = 0 , init_y = 0, init_heading = 0;
    public static double LPreload_x = 34, LPreload_y = -7.5, LPreload_heading = 90;
    public static double MPreload_x = 41, MPreload_y = 0, MPreload_heading = 90;
    public static double RPreload_x = 41, RPreload_y = 4, RPreload_heading = -0;

    public static double LBackdrop_x = 17.5, LBackdrop_y = -93.6, LBackdrop_heading = -90;
    public static double MBackdrop_x = 27.5, MBackdrop_y = -93.6, MBackdrop_heading = -90;
    public static double RBackdrop_x = 39, RBackdrop_y = -93.6, RBackdrop_heading = -90;
    public static double Park_x = 4, Park_y = -36, Park_heading = -91;
    private final Pose2d preloadPosL = new Pose2d(LPreload_x,LPreload_y,Math.toRadians(LPreload_heading));
    private Pose2d preloadPosM = new Pose2d(MPreload_x,MPreload_y,Math.toRadians(MPreload_heading));
    private Pose2d preloadPosR = new Pose2d(RPreload_x,RPreload_y,Math.toRadians(RPreload_heading));
    private Pose2d initPos = new Pose2d(init_x, init_y, Math.toRadians(init_heading));

    private Pose2d LbackdropPos = new Pose2d(LBackdrop_x,LBackdrop_y,Math.toRadians(LBackdrop_heading));
    private Pose2d MbackdropPos = new Pose2d(MBackdrop_x,MBackdrop_y,Math.toRadians(MBackdrop_heading));
    private Pose2d RbackdropPos = new Pose2d(RBackdrop_x,RBackdrop_y,Math.toRadians(RBackdrop_heading));
    private Pose2d parkPos = new Pose2d(Park_x,Park_y,Math.toRadians(Park_heading));
    private Pose2d stack = new Pose2d(51, 16, Math.toRadians(-90));
    private Pose2d backdropDetection = new Pose2d(26, -85, Math.toRadians(-90));
    private Pose2d againStack = new Pose2d(50, 16, Math.toRadians(-90));
    private Pose2d transitFar = new Pose2d(stack.getX() + 2, -60, Math.toRadians(-90));
    private Pose2d transitClose = new Pose2d(stack.getX() + 2, -80, Math.toRadians(-90));
    private Controls c;
    private AutoServo intakeServo;
    private NanoClock clock;
    private double intakeStart = 0;
    private boolean TO_INTAKE = false;
    private double startTime = 0;
    private boolean REVERSE = false;
    private boolean TO_RELOCALIZE = false;

    private double angle = 75;
    private double step = 4;
    private int level = 1;
    private double startDetection = 0;
    public boolean stationary = false;

    Pose2d STACK = stack;

    @SuppressLint("SuspiciousIndentation")
    public void runOpMode() throws InterruptedException{
        clock = NanoClock.system();

        c = new Controls(gamepad1, gamepad2);
        ControlHub ch = new ControlHub(hardwareMap);
        ExpansionHub eh = new ExpansionHub(hardwareMap);
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
        RedFarDetectionPipeline detector = new RedFarDetectionPipeline(telemetry, false);

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

        TrajectorySequence preloadLeft = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(preloadPosL)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0.65);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                })
                .forward(4)
                .lineToLinearHeading(stack)
                .addTemporalMarker(() -> {
                    TO_INTAKE = true;
                })
                .build();
        TrajectorySequence preloadMiddle = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(preloadPosM)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0.65);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                })
                .forward(4)
                .lineToLinearHeading(stack)
                .addTemporalMarker(() -> {
                    TO_INTAKE = true;
                })
                .build();
        TrajectorySequence preloadRight = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(preloadPosR)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0.65);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                })
                .forward(4)
                .lineToLinearHeading(stack)
                .addTemporalMarker(() -> {
                    TO_INTAKE = true;
                })
                .build();

        TrajectorySequence fromStackToBackboard = mecanumDrive.trajectorySequenceBuilder(stack)
                .waitSeconds(0.35)
                .addTemporalMarker(() -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0.65);
                    intakeServo.setAngle(90);
                })
                .addTemporalMarker(1, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                })
                .forward(70)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
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
                .splineToConstantHeading(new Vector2d(backdropDetection.getX(), backdropDetection.getY()), backdropDetection.getHeading())
                .addTemporalMarker(() -> {
                    OutTake.STATES.currentLevel = level;
                    level += 3;
                    Controls.ExtendElevator = true;
                })
                .addTemporalMarker(() -> {
                    TO_RELOCALIZE = true;
                })
                .build();

        TrajectorySequence bipbupToStackR = mecanumDrive.trajectorySequenceBuilder(RbackdropPos)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 2.54, DriveConstants.TRACK_WIDTH))
                .back(5)
                .lineToLinearHeading(transitClose)
                .splineToConstantHeading(new Vector2d(transitFar.getX(), transitFar.getY()), -transitClose.getHeading())
                .splineToConstantHeading(new Vector2d(againStack.getX(), againStack.getY()), -againStack.getHeading())
                .addTemporalMarker(() -> {
                    TO_INTAKE = true;
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence bipbupToStackM = mecanumDrive.trajectorySequenceBuilder(MbackdropPos)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 2.54, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(transitClose)
                .addTemporalMarker(() -> {
                    STACK = new Pose2d(againStack.getX() + 2, againStack.getY() + 2, againStack.getHeading());
                })
                .splineToConstantHeading(new Vector2d(transitFar.getX() + 4, transitFar.getY()), -transitClose.getHeading())
                .splineToConstantHeading(new Vector2d(againStack.getX() + 2, againStack.getY()), -againStack.getHeading())
                .addTemporalMarker(() -> {
                    TO_INTAKE = true;
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence bipbupToStackL = mecanumDrive.trajectorySequenceBuilder(LbackdropPos)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 2.54, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(transitClose)
                .addTemporalMarker(() -> {
                    STACK = new Pose2d(againStack.getX() + 2, againStack.getY() + 2, againStack.getHeading());
                })
                .splineToConstantHeading(new Vector2d(transitFar.getX() - 2, transitFar.getY()), -transitClose.getHeading())
                .splineToConstantHeading(new Vector2d(againStack.getX() - 1, againStack.getY()), -againStack.getHeading())
                .addTemporalMarker(() -> {
                    TO_INTAKE = true;
                })
                .waitSeconds(1)
                .build();

        TrajectorySequence yellow_R = mecanumDrive.trajectorySequenceBuilder(backdropDetection)
                .lineToLinearHeading(RbackdropPos)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                    STACK = againStack;
                })
                .waitSeconds(0.15)
                .addTemporalMarker(() -> {
                    Controls.RetractElevator = true;
                    if(clock.seconds() - startTime < 19) {
                        mecanumDrive.followTrajectorySequenceAsync(bipbupToStackR);
                    } else {
                        mecanumDrive.followTrajectorySequenceAsync(mecanumDrive.trajectorySequenceBuilder(mecanumDrive.getPoseEstimate()).lineToLinearHeading(backdropDetection).build());
                        stationary = true;
                    }
                })
                .build();

        TrajectorySequence yellow_M = mecanumDrive.trajectorySequenceBuilder(backdropDetection)
                .lineToLinearHeading(MbackdropPos)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                    STACK = againStack;
                })
                .waitSeconds(0.15)
                .addTemporalMarker(() -> {
                    Controls.RetractElevator = true;
                    if(clock.seconds() - startTime < 19) {
                        mecanumDrive.followTrajectorySequenceAsync(bipbupToStackM);
                    } else {
                        mecanumDrive.followTrajectorySequenceAsync(mecanumDrive.trajectorySequenceBuilder(mecanumDrive.getPoseEstimate()).lineToLinearHeading(backdropDetection).build());
                        stationary = true;
                    }
                })
                .build();

        TrajectorySequence yellow_L = mecanumDrive.trajectorySequenceBuilder(backdropDetection)
                .lineToLinearHeading(LbackdropPos)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                    STACK = againStack;
                })
                .waitSeconds(0.15)
                .addTemporalMarker(() -> {
                    Controls.RetractElevator = true;
                    if(clock.seconds() - startTime < 19) {
                        mecanumDrive.followTrajectorySequenceAsync(bipbupToStackL);
                    } else {
                        mecanumDrive.followTrajectorySequenceAsync(mecanumDrive.trajectorySequenceBuilder(mecanumDrive.getPoseEstimate()).lineToLinearHeading(backdropDetection).build());
                        stationary = true;
                    }
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
                mecanumDrive.followTrajectorySequenceAsync(preloadLeft);
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
                    telemetry.addData("pose_from_tag_y: ", OFFSET_Y - target_tag.pose.z * METERS_TO_INCHES);

                    Pose2d pose = mecanumDrive.getPoseEstimate();
                    telemetry.addData("pose_from_odo_x: ", pose.getX());
                    telemetry.addData("pose_from_odo_y: ", pose.getY());
                    mecanumDrive.setPoseEstimate(new Pose2d(OFFSET_X + target_tag.pose.x * METERS_TO_INCHES + offset_x, -OFFSET_Y + target_tag.pose.z * METERS_TO_INCHES, pose.getHeading()));

                    TO_RELOCALIZE = false;
                    new Thread(() -> {
                        camera.closeCameraDevice();
                    }).start();

                    if(caz == 3 && STACK == stack) {
                        mecanumDrive.followTrajectorySequenceAsync(yellow_R);
                    } else if(caz == 1 && STACK == stack) {
                        mecanumDrive.followTrajectorySequenceAsync(yellow_L);
                    } else {
                        mecanumDrive.followTrajectorySequenceAsync(yellow_M);
                    }

                    startDetection = 0;
                }


                telemetry.addLine("relocalization RUNNING");
                telemetry.update();
            }

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
                    TO_INTAKE = false;
                    mecanumDrive.followTrajectorySequenceAsync(fromStackToBackboard);
                    intakeStart = 0;
                } else if(!mecanumDrive.isBusy()) {
                    mecanumDrive.followTrajectorySequenceAsync(chipiChapa());
                }

            }
            if(stationary){
                Controls.RetractElevator = true;
            }
            outTake.update();
        }
    }

    public TrajectorySequence chipiChapa() {
        if(caz == 3) return mecanumDrive.trajectorySequenceBuilder(new Pose2d(STACK.getX(), STACK.getY(), STACK.getHeading())).forward(3).back(3).build();
        else return mecanumDrive.trajectorySequenceBuilder(STACK).forward(3).back(3).build();
    }
}