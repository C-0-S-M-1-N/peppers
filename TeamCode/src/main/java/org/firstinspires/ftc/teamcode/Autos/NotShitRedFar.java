package org.firstinspires.ftc.teamcode.Autos;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Autos.RedCloseTrajectory;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.OutTake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utils.AutoServo;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.net.IDN;

@Autonomous(group = "Autos", preselectTeleOp = "pipers \\uD83C\\uDF36", name = "NotShitAutoRedFar")
@Config
public class NotShitRedFar extends LinearOpMode {
    private org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive mecanumDrive;
    RedCloseTrajectory t;
    OpenCvCamera camera;

    int caz;
    public static double init_x = 0 , init_y = 0, init_heading = 0;
    public static double LPreload_x = 39, LPreload_y = 7, LPreload_heading = 0;
    public static double MPreload_x = 39, MPreload_y = 4, MPreload_heading = 90;
    public static double RPreload_x = 32, RPreload_y = -7, RPreload_heading = 90;

    public static double LBackdrop_x = 18, LBackdrop_y = 85, LBackdrop_heading = 90;
    public static double MBackdrop_x = 30, MBackdrop_y = 85, MBackdrop_heading = 90;
    public static double RBackdrop_x = 31, RBackdrop_y = 37.2, RBackdrop_heading = 90;
    public static double Park_x = 4, Park_y = 36, Park_heading = 91;
    public static double Transit_x = 51, Transit_y = 6, Transit_heading = 90;
    private final Pose2d preloadPosL = new Pose2d(LPreload_x,LPreload_y,Math.toRadians(LPreload_heading));
    private Pose2d preloadPosM = new Pose2d(MPreload_x,MPreload_y,Math.toRadians(MPreload_heading));
    private Pose2d preloadPosR = new Pose2d(RPreload_x,RPreload_y,Math.toRadians(RPreload_heading));
    private Pose2d initPos = new Pose2d(init_x, init_y, Math.toRadians(init_heading));

    private Pose2d LbackdropPos = new Pose2d(LBackdrop_x,LBackdrop_y,Math.toRadians(LBackdrop_heading));
    private Pose2d MbackdropPos = new Pose2d(MBackdrop_x,MBackdrop_y,Math.toRadians(MBackdrop_heading));
    private Pose2d RbackdropPos = new Pose2d(RBackdrop_x,RBackdrop_y,Math.toRadians(RBackdrop_heading));
    private Pose2d parkPos = new Pose2d(Park_x,Park_y,Math.toRadians(Park_heading));
    private Pose2d transitPos = new Pose2d(Transit_x, Transit_y, Math.toRadians(Transit_heading));
    private Controls c;
    private AutoServo intakeServo;

    @SuppressLint("SuspiciousIndentation")
    public void runOpMode() throws InterruptedException{
        c = new Controls(gamepad1, gamepad2);
        ControlHub ch = new ControlHub(hardwareMap);
        ExpansionHub eh = new ExpansionHub(hardwareMap);
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        t = new RedCloseTrajectory(mecanumDrive);
        intakeServo = new AutoServo(SERVO_PORTS.S3, true, false, 0, AutoServo.type.AXON);
        intakeServo.setPosition(0);

        OutTake outTake = new OutTake(hardwareMap, telemetry);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        OutTake.useControls = false;

        // ------------------ OpenCv initialisation code
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        ObjectDetectionPipeline detector = new ObjectDetectionPipeline(telemetry, false);

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
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0.6);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                })
                .lineToLinearHeading(transitPos)
                .addTemporalMarker(() -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, -1);
                    intakeServo.setAngle(11);
                })
                .back(33)
                .turn(Math.toRadians(20))
                .turn(Math.toRadians(-40))
                .turn(Math.toRadians(20))
                .addTemporalMarker(() -> {
                    intakeServo.setAngle(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                })
                .addTemporalMarker(() -> {
                    outTake.LeftClaw.forceClose = true;
                    outTake.RightClaw.forceClose = true;
                })
                .forward(85)
                .addTemporalMarker(() -> {
                    OutTake.STATES.currentLevel = 2;
                    Controls.ExtendElevator = true;
                    outTake.LeftClaw.forceClose = false;
                    outTake.RightClaw.forceClose = false;
                })
                .lineToLinearHeading(LbackdropPos)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                })
                .waitSeconds(0.15)
                .addTemporalMarker(() -> {
                    Controls.RetractElevator = true;
                })
                .lineToLinearHeading(new Pose2d(53, 76, Math.toRadians(90)))
                .UNSTABLE_addTemporalMarkerOffset(2, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, -1);
                    intakeServo.setAngle(18);
                })
                .lineToLinearHeading(new Pose2d(48, -26, Math.toRadians(90)))
                .turn(Math.toRadians(20))
                .addTemporalMarker(() -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, -1);
                    intakeServo.setAngle(0);
                })
                .turn(Math.toRadians(-40))
                .addTemporalMarker(() -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, -1);
                    intakeServo.setAngle(18);
                })
                .turn(Math.toRadians(20))
                .waitSeconds(1)
                .addTemporalMarker(() -> {
                    intakeServo.setAngle(0);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                })
                .addTemporalMarker(() -> {
                    outTake.LeftClaw.forceClose = true;
                    outTake.RightClaw.forceClose = true;
                })
                .forward(85)
                .addTemporalMarker(() -> {
                    OutTake.STATES.currentLevel = 3;
                    Controls.ExtendElevator = true;
                    outTake.LeftClaw.forceClose = false;
                    outTake.RightClaw.forceClose = false;
                })
                .lineToLinearHeading(MbackdropPos)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                })
                .waitSeconds(0.15)
                .addTemporalMarker(() -> {
                    Controls.RetractElevator = true;
                })
                .build();
        TrajectorySequence preloadMiddle = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(preloadPosM)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    OutTake.STATES.currentLevel = 2;
                    Controls.ExtendElevator = true;
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0.6);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                })
                .lineToLinearHeading(MbackdropPos)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                })
                .waitSeconds(0.15)
                .addTemporalMarker(() -> {
                    Controls.RetractElevator = true;
                })
                .lineToLinearHeading(parkPos)
                .build();
        TrajectorySequence preloadRight = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(preloadPosR)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    OutTake.STATES.currentLevel = 2;
                    Controls.ExtendElevator = true;
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0.6);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                })
                .lineToLinearHeading(RbackdropPos)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                })
                .waitSeconds(0.15)
                .addTemporalMarker(() -> {
                    Controls.RetractElevator = true;
                })
                .lineToLinearHeading(parkPos)
                .build();

        waitForStart();

        switch (detector.getLocation()) {
            case LEFT:
                telemetry.addLine("LEFT CASE");
                caz = 1;
                mecanumDrive.followTrajectorySequenceAsync(preloadLeft);
                break;
            case MIDDLE:
                telemetry.addLine("MIDDLE CASE");
                caz = 2;
                mecanumDrive.followTrajectorySequenceAsync(preloadLeft);

                break;
            case RIGHT:
                telemetry.addLine("RIGHT CASE");
                caz = 3;
                mecanumDrive.followTrajectorySequenceAsync(preloadLeft);
                break;
            default:
                // not used
                telemetry.addData("location" , "nu stiu boss");
                mecanumDrive.followTrajectorySequenceAsync(preloadLeft);
                break;
        }

        new Thread(() -> {
            camera.stopStreaming();
        });

        while(opModeIsActive() && !isStopRequested()){
            mecanumDrive.update();
            outTake.update();
            outTake.runTelemetry();
            c.loop();
            intakeServo.update();
        }
    }}