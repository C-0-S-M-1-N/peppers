package org.firstinspires.ftc.teamcode.Autos;

import static java.lang.Double.min;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.Localizer;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Components.Grippers;
import org.firstinspires.ftc.teamcode.Parts.MecanumDrive;
import org.firstinspires.ftc.teamcode.Parts.OutTake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utils.AutoServo;
import org.firstinspires.ftc.teamcode.utils.HoldPosition;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(group = "Autos", preselectTeleOp = "pipers \\uD83C\\uDF36", name = "CustomBlueClose")
@Config
public class CustomBlueClose extends LinearOpMode {
    OpenCvCamera camera;

    int caz;
    public static double init_x = 0 , init_y = 0, init_heading = 0;
    public static double LPreload_x = 35, LPreload_y = 15, LPreload_heading = 90;
    public static double MPreload_x = 39, MPreload_y = 4, MPreload_heading = 90;
    public static double RPreload_x = 33, RPreload_y = -7, RPreload_heading = 90;

    public static double LBackdrop_x = 17, LBackdrop_y = 36, LBackdrop_heading = 90;
    public static double MBackdrop_x = 26, MBackdrop_y = 36, MBackdrop_heading = 90;
    public static double RBackdrop_x = 33.9, RBackdrop_y = 36, RBackdrop_heading = 90;
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
    private Pose2d preTruss = new Pose2d(6, 20, Math.toRadians(90));
    private Pose2d preStack = new Pose2d(6, -60, Math.toRadians(90));
    private Pose2d stack = new Pose2d(29.5, -75.5, Math.toRadians(90));
    private Pose2d start, end;
    private AutoServo intakeServo;
    double intakeStart = 0;
    public double startTime = 0;
    NanoClock clock;
    HoldPosition holdPos;
    MecanumDrive drive;
    boolean ELEVATOR_UP = false;

    public static double POSE_THRESHOLD = 1;
    public static double PURPLE_PRELOAD_TIME = 3;
    public static double YELLOW_PRELOAD_TIME = 3;
    public static double TRAVEL_TRUSS_TIME = 5;

    enum ROBOT_STATE {
        PRELOAD_PURPLE,
        PRELOAD_YELLOW,
        PRETRUSS,
        TRAVEL_TRUSS,
        PRESTACK,
        STACK,
        INTAKE,
        REVERSE,
        PARK,
    }

    ROBOT_STATE STATE = ROBOT_STATE.PRELOAD_PURPLE;
    public Pose2d DESIRED_POSE = new Pose2d();

    @SuppressLint("SuspiciousIndentation")
    public void runOpMode() throws InterruptedException{
        clock = NanoClock.system();
        ControlHub ch = new ControlHub(hardwareMap);
        ExpansionHub eh = new ExpansionHub(hardwareMap);
        Controls c = new Controls(gamepad1, gamepad2);
        intakeServo = new AutoServo(SERVO_PORTS.S3, true, false, 0, AutoServo.type.AXON);
        intakeServo.setAngle(10);

        OutTake outTake = new OutTake(hardwareMap, telemetry);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        OutTake.useControls = false;

        drive = new MecanumDrive(telemetry, hardwareMap);
        holdPos = new HoldPosition(telemetry, hardwareMap);
        holdPos.holdPos(0, 0, 0);

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

        waitForStart();

        switch (detector.getLocation()) {
            case LEFT:
                telemetry.addLine("LEFT CASE");
                caz = 1;
                DESIRED_POSE = preloadPosL;
                break;
            case MIDDLE:
                telemetry.addLine("MIDDLE CASE");
                caz = 2;
                DESIRED_POSE = preloadPosM;
                break;
            case RIGHT:
                telemetry.addLine("RIGHT CASE");
                caz = 3;
                DESIRED_POSE = preloadPosR;
                break;
            default:
                // not used
                telemetry.addData("location" , "nu stiu boss");
                DESIRED_POSE = preloadPosR;
                break;
        }

        new Thread(() -> {
            camera.stopStreaming();
        }).start();

        while(opModeIsActive() && !isStopRequested()) {

            outTake.update();
            outTake.runTelemetry();
            c.loop();
            intakeServo.update();

            double t, x, y, h;

            switch (STATE) {
                case PRELOAD_PURPLE:
                    if(startTime == 0) {
                        startTime = clock.seconds();
                        start = new Pose2d();
                        end = DESIRED_POSE;
                    }

                    t = min(1, (clock.seconds() - startTime) / PURPLE_PRELOAD_TIME);

                    x = (1 - t) * start.getX() + t * end.getX();
                    y = (1 - t) * start.getY() + t * end.getY();
                    h = (1 - t) * start.getHeading() + t * end.getHeading();

                    DESIRED_POSE = new Pose2d(x, y, h);

                    if(holdPos.getDistFromTarget() < POSE_THRESHOLD && t > 1) {
                        start = end;

                        if(DESIRED_POSE == preloadPosL) DESIRED_POSE = LbackdropPos;
                        if(DESIRED_POSE == preloadPosM) DESIRED_POSE = MbackdropPos;
                        if(DESIRED_POSE == preloadPosR) DESIRED_POSE = RbackdropPos;

                        end = DESIRED_POSE;

                        STATE = ROBOT_STATE.PRELOAD_YELLOW;

                        startTime = 0;
                    }
                    break;
                case PRELOAD_YELLOW:
                    if(startTime == 0) {
                        startTime = clock.seconds();

                        OutTake.STATES.currentLevel = 1;
                        Controls.ExtendElevator = true;
                        ELEVATOR_UP = true;
                        ControlHub.setMotorPower(MOTOR_PORTS.M2, 0.5);
                    }

                    t = min(1, (clock.seconds() - startTime) / YELLOW_PRELOAD_TIME);

                    x = (1 - t) * start.getX() + t * end.getX();
                    y = (1 - t) * start.getY() + t * end.getY();
                    h = (1 - t) * start.getHeading() + t * end.getHeading();

                    DESIRED_POSE = new Pose2d(x, y, h);

                    if(clock.seconds() - startTime > 0.3) {
                        ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                    }

                    if(holdPos.getDistFromTarget() < POSE_THRESHOLD && t > 1) {
                        STATE = ROBOT_STATE.PRETRUSS;

                        startTime = clock.seconds();
                        Controls.DropLeft = true;
                        Controls.DropRight = true;
                    }
                    break;
                case PRETRUSS:
                    if(holdPos.getDistFromTarget() < 1 && ELEVATOR_UP == false) {
                        startTime = clock.seconds();
                        STATE = ROBOT_STATE.TRAVEL_TRUSS;
                    }

                    if(clock.seconds() - startTime > 0.15 && ELEVATOR_UP) {
                        Controls.RetractElevator = true;
                        ELEVATOR_UP = false;
                        DESIRED_POSE = preTruss;
                    }
                    break;
                case TRAVEL_TRUSS:
                    t = min(1, (clock.seconds() - startTime) / TRAVEL_TRUSS_TIME);

                    x = (1 - t) * preTruss.getX() + t * preStack.getX();
                    y = (1 - t) * preTruss.getY() + t * preStack.getY();
                    h = (1 - t) * preTruss.getHeading() + t * preStack.getHeading();

                    DESIRED_POSE = new Pose2d(x, y, h);

                    if(clock.seconds() - startTime > TRAVEL_TRUSS_TIME && holdPos.getDistFromTarget() < POSE_THRESHOLD) {
                        STATE = ROBOT_STATE.PRESTACK;
                        DESIRED_POSE = preStack;
                        startTime = clock.seconds();
                    }
                    break;
                case PRESTACK:
                    intakeServo.setAngle(30);

                    DESIRED_POSE = stack;
                    break;
            }

            holdPos.holdPos(DESIRED_POSE.getX(), DESIRED_POSE.getY(), DESIRED_POSE.getHeading());
            double[] arr = holdPos.update();
            drive.update(arr[0], arr[1], arr[2] > 0 ? arr[2] : 0, arr[2] < 0 ? -arr[2] : 0, true);
        }
    }
}