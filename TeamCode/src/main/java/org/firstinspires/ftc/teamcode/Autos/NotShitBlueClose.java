package org.firstinspires.ftc.teamcode.Autos;

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
import org.firstinspires.ftc.teamcode.Components.Grippers;
import org.firstinspires.ftc.teamcode.Parts.OutTake;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utils.AutoServo;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(group = "Autos", preselectTeleOp = "pipers \\uD83C\\uDF36", name = "NotShitBlueClose")
@Config
public class NotShitBlueClose extends LinearOpMode {
    private org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable mecanumDrive;
    RedCloseTrajectory t;
    OpenCvCamera camera;

    int caz;
    public static double init_x = 0 , init_y = 0, init_heading = 0;
    public static double LPreload_x = 35, LPreload_y = 15, LPreload_heading = 90;
    public static double MPreload_x = 39, MPreload_y = 4, MPreload_heading = 90;
    public static double RPreload_x = 33, RPreload_y = -7, RPreload_heading = 90;

    public static double LBackdrop_x = 17, LBackdrop_y = 37.5, LBackdrop_heading = 90;
    public static double MBackdrop_x = 28, MBackdrop_y = 37.5, MBackdrop_heading = 90;
    public static double RBackdrop_x = 33.9, RBackdrop_y = 37.5, RBackdrop_heading = 90;
    public static double Park_x = -3, Park_y = 36, Park_heading = 91;
    public static double Transit_x = 11, Transit_y = -2, Transit_heading = 0.1;
    private final Pose2d preloadPosL = new Pose2d(LPreload_x,LPreload_y,Math.toRadians(LPreload_heading));
    private Pose2d preloadPosM = new Pose2d(MPreload_x,MPreload_y,Math.toRadians(MPreload_heading));
    private Pose2d preloadPosR = new Pose2d(RPreload_x,RPreload_y,Math.toRadians(RPreload_heading));
    private Pose2d initPos = new Pose2d(init_x, init_y, Math.toRadians(init_heading));

    private Pose2d LbackdropPos = new Pose2d(LBackdrop_x,LBackdrop_y,Math.toRadians(LBackdrop_heading));
    private Pose2d MbackdropPos = new Pose2d(MBackdrop_x,MBackdrop_y,Math.toRadians(MBackdrop_heading));
    private Pose2d RbackdropPos = new Pose2d(RBackdrop_x,RBackdrop_y,Math.toRadians(RBackdrop_heading));
    private Pose2d parkPos = new Pose2d(Park_x,Park_y,Math.toRadians(Park_heading));
    private Pose2d prePreTruss = new Pose2d(10, 14, Math.toRadians(90));
    private Pose2d preTruss = new Pose2d(10, 0, Math.toRadians(90));
    private Pose2d preStack = new Pose2d(10, -70, Math.toRadians(90));
    private Pose2d prePreStack = new Pose2d(10, -60, Math.toRadians(90));
    private Pose2d stack = new Pose2d(31, -75, Math.toRadians(90));
    private Controls c;
    private AutoServo intakeServo;
    public TrajectorySequence preloadLeft, preloadMiddle, preloadRight, fromStackToBackboard, fromBackboardToStack, chipichipa;
    public boolean TO_INTAKE = false;
    public boolean REVERSE = false;
    double intakeStart = 0;
    public double startTime = 0;
    NanoClock clock;

    @SuppressLint("SuspiciousIndentation")
    public void runOpMode() throws InterruptedException{
        clock = NanoClock.system();
        c = new Controls(gamepad1, gamepad2);
        ControlHub ch = new ControlHub(hardwareMap);
        ExpansionHub eh = new ExpansionHub(hardwareMap);
        mecanumDrive = new SampleMecanumDriveCancelable(hardwareMap);
        t = new RedCloseTrajectory(mecanumDrive);
        intakeServo = new AutoServo(SERVO_PORTS.S3, true, false, 0, AutoServo.type.AXON);
        intakeServo.setAngle(10);

        OutTake outTake = new OutTake(hardwareMap, telemetry);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        OutTake.useControls = false;

        // ------------------ OpenCv initialisation code
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        ObjectDetectionPipeline detector = new ObjectDetectionPipeline(telemetry, true);

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


        preloadLeft = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(preloadPosL)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    OutTake.STATES.currentLevel = 1;
                    Controls.ExtendElevator = true;
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
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
                .lineToLinearHeading(prePreTruss)
                .waitSeconds(0.001)
                .splineToConstantHeading(new Vector2d(preTruss.getX(), preTruss.getY()), -preTruss.getHeading())
                .splineToConstantHeading(new Vector2d(prePreStack.getX(), prePreStack.getY()), -prePreStack.getHeading())
                .splineToConstantHeading(new Vector2d(preStack.getX(), preStack.getY()), -preStack.getHeading())
                .addTemporalMarker(()-> {
                    intakeServo.setAngle(35);
                })
                .lineToLinearHeading(stack)
                .addTemporalMarker(() -> {
                    intakeServo.setAngle(90);
                })
                .waitSeconds(0.05)
                .addTemporalMarker(() -> {
                    TO_INTAKE = true;
                })
                .strafeRight(3)
                .build();
        preloadMiddle = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(preloadPosM)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    OutTake.STATES.currentLevel = 1;
                    Controls.ExtendElevator = true;
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
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
                .lineToLinearHeading(prePreTruss)
                .waitSeconds(0.001)
                .splineToConstantHeading(new Vector2d(preTruss.getX(), preTruss.getY()), -preTruss.getHeading())
                .splineToConstantHeading(new Vector2d(prePreStack.getX(), prePreStack.getY()), -prePreStack.getHeading())
                .splineToConstantHeading(new Vector2d(preStack.getX(), preStack.getY()), -preStack.getHeading())
                .addTemporalMarker(()-> {
                    intakeServo.setAngle(35);
                })
                .lineToLinearHeading(stack)
                .addTemporalMarker(() -> {
                    intakeServo.setAngle(90);
                })
                .waitSeconds(0.05)
                .addTemporalMarker(() -> {
                    TO_INTAKE = true;
                })
                .strafeRight(3)
                .build();
         preloadRight = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(preloadPosR)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    OutTake.STATES.currentLevel = 1;
                    Controls.ExtendElevator = true;
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
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
                 .lineToLinearHeading(prePreTruss)
                 .waitSeconds(0.0001)
                 .splineToConstantHeading(new Vector2d(preTruss.getX(), preTruss.getY()), -preTruss.getHeading())
                 .splineToConstantHeading(new Vector2d(prePreStack.getX(), prePreStack.getY()), -prePreStack.getHeading())
                 .splineToConstantHeading(new Vector2d(preStack.getX(), preStack.getY()), -preStack.getHeading())
                 .addTemporalMarker(()-> {
                     intakeServo.setAngle(35);
                 })
                 .lineToLinearHeading(stack)
                 .addTemporalMarker(() -> {
                     intakeServo.setAngle(90);
                 })
                 .waitSeconds(0.05)
                 .addTemporalMarker(() -> {
                     TO_INTAKE = true;
                 })
                 .strafeRight(3)
                 .build();

         fromStackToBackboard = mecanumDrive.trajectorySequenceBuilder(preloadLeft.end())
                 .addTemporalMarker(() -> {
                     ControlHub.setMotorPower(MOTOR_PORTS.M2, 0.75);
                 })
                 .UNSTABLE_addTemporalMarkerOffset(1.5, () -> {
                     ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                 })
                 .lineToLinearHeading(prePreStack)
                 .waitSeconds(0.0001)
                 .splineToConstantHeading(new Vector2d(preTruss.getX(), preTruss.getY()), preTruss.getHeading())
                 .splineToConstantHeading(new Vector2d(prePreTruss.getX(), prePreTruss.getY()), prePreTruss.getHeading())
                .addTemporalMarker(() -> {
                    OutTake.STATES.currentLevel = 4;
                    Controls.ExtendElevator = true;
                })
                .splineToLinearHeading(MbackdropPos, MbackdropPos.getHeading())
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                })
                 .waitSeconds(0.15)
                 .addTemporalMarker(() -> {
                     Controls.RetractElevator = true;
                 })
                 .addTemporalMarker(() -> {
                     if(clock.seconds() - startTime < 20)
                        mecanumDrive.followTrajectorySequenceAsync(fromBackboardToStack);
                     else
                         mecanumDrive.followTrajectoryAsync(mecanumDrive.trajectoryBuilder(mecanumDrive.getPoseEstimate()).lineToLinearHeading(parkPos).build());
                 })
                .build();

         fromBackboardToStack = mecanumDrive.trajectorySequenceBuilder(fromStackToBackboard.end())
                 .lineToLinearHeading(prePreTruss)
                 .splineToConstantHeading(new Vector2d(preTruss.getX(), preTruss.getY()), -preTruss.getHeading())
                 .splineToConstantHeading(new Vector2d(prePreStack.getX(), prePreStack.getY()), -prePreStack.getHeading())
                 .addTemporalMarker(()-> {
                     intakeServo.setAngle(90);
                 })
                 .splineToLinearHeading(stack, -stack.getHeading())
                 .addTemporalMarker(() -> {
                     TO_INTAKE = true;
                 })
                 .build();

         chipichipa = mecanumDrive.trajectorySequenceBuilder(stack)
                 .turn(Math.toRadians(-10))
                 .turn(Math.toRadians(20))
                 .turn(Math.toRadians(-10))
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
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0.55);
                    intakeServo.setAngle(90);
                } else {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, -1);
                    intakeServo.setAngle(40);
                }

                if(clock.seconds() - intakeStart > 1) {
                    REVERSE = !REVERSE;
                    intakeStart = clock.seconds();
                }
                if(OutTake.fullPixel() || clock.seconds() - startTime > 25) {
                    TO_INTAKE = false;
                    mecanumDrive.followTrajectorySequenceAsync(fromStackToBackboard);
                    intakeStart = 0;
                } else if(!mecanumDrive.isBusy()) {
                    //mecanumDrive.followTrajectorySequenceAsync(chipichipa);
                }

            }
        }
    }}