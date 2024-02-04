package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.Autos.RedFar.STATE.NULL;
import static org.firstinspires.ftc.teamcode.Autos.RedFar.STATE.PARK;
import static org.firstinspires.ftc.teamcode.Autos.RedFar.STATE.TAKE;
import static org.firstinspires.ftc.teamcode.Autos.RedFar.STATE.TO_BACKDROP;
import static org.firstinspires.ftc.teamcode.Autos.RedFar.STATE.TO_STACK;

import android.graphics.Camera;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.constraints.MecanumVelocityConstraint;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Parts.MecanumDrive;
import org.firstinspires.ftc.teamcode.Parts.OutTake;

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

@Autonomous(name = "redFar")
@Config
public class RedFar extends LinearOpMode {

    public enum STATE{
        TO_BACKDROP,
        TO_STACK,
        TAKE,
        PARK,
        NULL
    }
    STATE state;

    public static OpenCvCamera camera;
    private void initCamera(){
        int cameraId = hardwareMap.appContext.getResources().
                getIdentifier("cameraID", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().
                createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraId);
        ObjectDetectionPipelineRED pipe = new ObjectDetectionPipelineRED(telemetry, false);
        camera.setPipeline(pipe);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(432, 240, OpenCvCameraRotation.SENSOR_NATIVE);
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addLine("OpenCV camera failed");
            }
        });

        camera.stopStreaming();
    }

    public static OutTake outTake;
    public static SampleMecanumDriveCancelable drive;
    public static AutoServo intakeServo;
    public static Controls c;
    public static double step = 10, ANGLE = 29;
    private double angle = ANGLE;
    public static double backdropX = 30.8, backdropY = 97;
    boolean park = false;
    public static double stackX = 52, stackY = -17.2;
    public static double waitTime = 0.05;

    private ElapsedTime time;
    public  Pose2d preloadPosL = new Pose2d(40.4, -7, Math.toRadians(0));
    public  Pose2d transitPos = new Pose2d(50.49, 13, Math.toRadians(90));
    public static double strafeDist = 0;
    public static double DRIFT_OFFSET = 0;
    public boolean takingPixels = false;
    //-18.5
    public  Pose2d stackPosition = new Pose2d(stackX, stackY, Math.toRadians(90));
    public Pose2d stackPositionBack = new Pose2d(stackX - strafeDist , stackY, Math.toRadians(90));
    public  Pose2d preBackdropPos = new Pose2d(47.5, 72, Math.toRadians(90));
    public  Pose2d LbackdropPos = new Pose2d(backdropX, backdropY, Math.toRadians(90));
    private NanoClock clock;

    @Override
    public void runOpMode() throws InterruptedException{
        clock = NanoClock.system();
        ControlHub ch = new ControlHub(hardwareMap);
        ExpansionHub e = new ExpansionHub(hardwareMap);

//        initCamera();
        outTake = new OutTake(hardwareMap, telemetry);
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        intakeServo = new AutoServo(SERVO_PORTS.S3, true, false, 0, AutoServo.type.AXON);
        c = new Controls(gamepad1, gamepad2);
        time = new ElapsedTime();
        OutTake.STATES.currentLevel = 1;
        OutTake.bedAngle = 120;
        state = NULL;

        TrajectorySequence takePixels = drive.trajectorySequenceBuilder(stackPosition)
                .addTemporalMarker(() -> {
                    intakeServo.setAngle(angle);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, -1);
                    angle += step;
                    if(angle > 90) angle = 90;
                })
                .forward(1)
                .lineToLinearHeading(new Pose2d(stackPosition.getX()+3, stackPosition.getY(), stackPosition.getHeading() + Math.toRadians(10)))
                .lineToLinearHeading(new Pose2d(stackPosition.getX(), stackPosition.getY(), stackPosition.getHeading() - Math.toRadians(10)))
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0.76);
                    intakeServo.setAngle(1);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                })
                .addTemporalMarker(() -> {
                    intakeServo.setAngle(angle);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, -1);
                    angle += step;
                    if(angle > 90) angle = 90;
                })
                .addTemporalMarker(() -> {
                    state = TO_BACKDROP;
                })
                .build();

        TrajectorySequence toStackFromBackdrop = drive.trajectorySequenceBuilder(LbackdropPos)
                .addTemporalMarker(() -> {
                    Pose2d pose = drive.getPoseEstimate();

                    drive.setPoseEstimate(new Pose2d(pose.getX() + DRIFT_OFFSET, pose.getY(), pose.getHeading()));
                })
                .lineToSplineHeading(new Pose2d(preBackdropPos.getX(), preBackdropPos.getY(), preBackdropPos.getHeading()))
                .splineToLinearHeading(new Pose2d(stackPositionBack.getX(),stackPositionBack.getY(), stackPositionBack.getHeading()), -stackPositionBack.getHeading())
                //.lineToLinearHeading(new Pose2d(stackPositionBack.getX(),stackPositionBack.getY(), stackPositionBack.getHeading()))
                .addTemporalMarker(() -> {
                    state = TAKE;
                })


                .build();
        TrajectorySequence toBackdropFromStack = drive.trajectorySequenceBuilder(stackPosition)
                .addTemporalMarker(() -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0.7);
                })
                .waitSeconds(0.03)
                .addTemporalMarker(() -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                })
                //.lineToLinearHeading(preBackdropPos)
                .lineToSplineHeading(new Pose2d(preBackdropPos.getX() + 5, preBackdropPos.getY()-40, preBackdropPos.getHeading()))
                .splineToSplineHeading(new Pose2d(preBackdropPos.getX(), preBackdropPos.getY(), preBackdropPos.getHeading()), preBackdropPos.getHeading())
                .addTemporalMarker(() -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                })
                .addTemporalMarker(() -> {
                    Controls.ExtendElevator = true;
                    OutTake.STATES.currentLevel ++;
                })

                .splineToSplineHeading(new Pose2d(LbackdropPos.getX(), LbackdropPos.getY(), LbackdropPos.getHeading()), Math.toRadians(90))
                .waitSeconds(waitTime)
                .addTemporalMarker(() -> {
                    Controls.DropLeft = true;
                    Controls.DropRight = true;
                })
                .waitSeconds(waitTime)
                .addTemporalMarker(() -> {
                    Controls.RetractElevator = true;
                    if(time.seconds()<=25)
                        state = TO_STACK;
                    else
                        state = PARK;
                })
                .build();

        TrajectorySequence toStackFromLeftPixel = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(preloadPosL)
                .addTemporalMarker(() -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0.5);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                })
//                .lineToLinearHeading(transitPos)
                .lineToLinearHeading(stackPosition)
//                .splineTo(new Vector2d(stackPosition.getX(), stackPosition.getY()), stackPosition.getHeading())
                .addTemporalMarker(() -> {
                    state = TAKE;
                })
                .build();

        waitForStart();

        double startTime = clock.seconds();
        time.reset();
        drive.followTrajectorySequenceAsync(toStackFromLeftPixel);

        while (opModeIsActive()){
            if(takingPixels && OutTake.fullPixel()){
                takingPixels = false;
                drive.breakFollowing();
                state = TO_BACKDROP;
            }
            switch (state){
                case TO_STACK:
                    if(time.seconds() <= 25) {
                        drive.followTrajectorySequenceAsync(toStackFromBackdrop);
                    }
                    state = NULL;
                    break;
                case TO_BACKDROP:
                    if(OutTake.fullPixel()){
                        drive.followTrajectorySequenceAsync(toBackdropFromStack);
                        intakeServo.setAngle(0);
                    } else {
                        drive.followTrajectorySequenceAsync(takePixels);
                    }
                    state = NULL;
                    break;
                case TAKE:
                    takingPixels = true;
                    drive.followTrajectorySequenceAsync(takePixels);
                    if(OutTake.fullPixel() || (time.seconds() > 26)) {
                        drive.breakFollowing();
                        drive.followTrajectorySequenceAsync(toBackdropFromStack);
                    }
                    state = NULL;
                    break;
                case PARK:
                    break;

            }


            outTake.update();
            intakeServo.update();
            c.loop();
            drive.update();

            //outTake.runTelemetry();
            FtcDashboard.getInstance().getTelemetry().addData("time", time.seconds());
            FtcDashboard.getInstance().getTelemetry().update();
        }
        OutTake.bedAngle = 60;

    }
}
