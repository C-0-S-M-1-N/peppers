package org.firstinspires.ftc.teamcode.Autos;

import static org.firstinspires.ftc.teamcode.Autos.RedFar.STATE.NULL;
import static org.firstinspires.ftc.teamcode.Autos.RedFar.STATE.TAKE;
import static org.firstinspires.ftc.teamcode.Autos.RedFar.STATE.TO_BACKDROP;
import static org.firstinspires.ftc.teamcode.Autos.RedFar.STATE.TO_STACK;

import android.graphics.Camera;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
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

@Autonomous(name = "redFar")
@Config
public class RedFar extends LinearOpMode {

    public enum STATE{
        TO_BACKDROP,
        TO_STACK,
        TAKE,
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
    public static SampleMecanumDrive drive;
    public static AutoServo intakeServo;
    public static Controls c;
    public static double step = 10, angle = 35;

    private ElapsedTime time;
    public static Pose2d
            preloadPosL = new Pose2d(30.64, 3.5, Math.toRadians(90)),
            transitPos = new Pose2d(50.49, 7, Math.toRadians(90)),
            stackPosition = new Pose2d(48.1, -18.5, Math.toRadians(90)),
            preBackdropPos = new Pose2d(50, 62.65, Math.toRadians(90)),
            LbackdropPos = new Pose2d(35.24, 94.13, Math.toRadians(90));

    @Override
    public void runOpMode() throws InterruptedException{
        ControlHub ch = new ControlHub(hardwareMap);
        ExpansionHub e = new ExpansionHub(hardwareMap);

//        initCamera();
        outTake = new OutTake(hardwareMap, telemetry);
        drive = new SampleMecanumDrive(hardwareMap);
        intakeServo = new AutoServo(SERVO_PORTS.S3, true, false, 0, AutoServo.type.AXON);
        c = new Controls(gamepad1, gamepad2);
        time = new ElapsedTime();
        OutTake.STATES.currentLevel = 1;
        state = NULL;



        TrajectorySequence LeftCase = drive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(preloadPosL)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0.45);
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                })
                .forward(2)
                .lineToLinearHeading(transitPos)
                .lineToLinearHeading(stackPosition)
                .addTemporalMarker(() -> {
                    state = TAKE;
                })
                .build();
        telemetry.update();
        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(LbackdropPos)
                .lineToLinearHeading(preBackdropPos)
                .lineToLinearHeading(stackPosition)
                .addTemporalMarker(() -> {
                    state = TAKE;
                })
                .build();

        TrajectorySequence takePixels = drive.trajectorySequenceBuilder(stackPosition)
                .addTemporalMarker(() -> {
                    intakeServo.setAngle(angle);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, -1);
                    angle += step;
                })
                .turn(Math.toRadians(10))
                .turn(Math.toRadians(-20))
                .turn(Math.toRadians(10))
                .addTemporalMarker(() -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0.6);
                    intakeServo.setAngle(1);
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                })
                .addTemporalMarker(() -> {
                    state = TO_BACKDROP;
                })
                .build();

        TrajectorySequence goToBackdrop = drive.trajectorySequenceBuilder(stackPosition)
                .addTemporalMarker(() -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0.5);
                    intakeServo.setAngle(0);
                })
                .lineToLinearHeading(preBackdropPos)
                .addTemporalMarker(() -> {
                    Controls.ExtendElevator = true;
                    OutTake.STATES.currentLevel ++;
                })
                .addTemporalMarker(() -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                })
                .waitSeconds(0.1)
                .lineToLinearHeading(LbackdropPos)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                })
                .waitSeconds(0.3)
                .addTemporalMarker(() -> {
                    Controls.RetractElevator = true;
                    state = TO_STACK;
                })
                .build();

        TrajectorySequence toStackFromBackdrop = drive.trajectorySequenceBuilder(LbackdropPos)
                .splineToSplineHeading(stackPosition, Math.toRadians(10))

                .build();

        waitForStart();
        time.reset();
        drive.followTrajectorySequenceAsync(LeftCase);


        while (opModeIsActive()){
            switch (state){
                case TO_STACK:
                    drive.followTrajectorySequenceAsync(goToStack);
                    state = NULL;
                    break;
                case TO_BACKDROP:
                    if(outTake.fullPixel()){
                        drive.followTrajectorySequenceAsync(goToBackdrop);
                    } else {
                        drive.followTrajectorySequenceAsync(takePixels);
                    }
                    state = NULL;
                    break;
                case TAKE:
                    drive.followTrajectorySequenceAsync(takePixels);
                    state = NULL;
                    break;
            }


            outTake.update();
            intakeServo.update();
            c.loop();
            drive.update();

            outTake.runTelemetry();
            telemetry.update();
        }

    }
}
