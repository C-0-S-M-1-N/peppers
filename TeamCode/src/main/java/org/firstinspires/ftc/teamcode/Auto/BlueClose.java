package org.firstinspires.ftc.teamcode.Auto;

import static java.lang.Math.min;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.BlueCloseDetectionPipeline;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Components.Grippers;
import org.firstinspires.ftc.teamcode.Components.Hang;
import org.firstinspires.ftc.teamcode.Components.OutTakeExtension;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.OutTake;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous(name = "blueClose")
@Config
public class BlueClose extends LinearOpMode {
    enum State{
        INTAKE,
        OTHER_INTAKE,
        NOT_INTAKE
    }
    enum CASE {
        LEFT,
        MIDDLE,
        RIGHT
    }

    private CASE caz = CASE.LEFT;

    private State state = State.NOT_INTAKE;
    private boolean relocalize = false;

    public static double middlePurple_x = 21.5, middlePurple_y = 1.3,
            middleYellow_x = 23, middleYellow_y = 31, middleYellow_h = Math.toRadians(70),
    stack_x = 23, stack_y = -76.5,
    prebackdropMiddle_x = 25.24, prebackdropMiddle_y = 6,
    leftpurple_x = 14.5, leftpurple_y = -7, leftpurple_h = -0.44,
    leftyellow_x = 29, leftyellow_y = 32, leftyellow_h = Math.toRadians(70),
            rightpurple_x = 11.6, rightpurple_y = 3, rightpurple_h = -6,
            rightyellow_x = 17, rightyellow_y = 31, rightyellow_h = Math.toRadians(70),
    transit_x = 3, transit_y = -2, transit_h = Math.toRadians(90),
    angledStack_x = 21, angledStack_y = -76, angledStack_h = Math.toRadians(115);

    public static Pose2d middlePurple = new Pose2d(middlePurple_x, middlePurple_y, 0),
    middleYellow = new Pose2d(middleYellow_x, middleYellow_y, middleYellow_h);
    boolean readSensor = false;
    private TrajectorySequence toAngledStack, otherToAngledStack;

    private int stackPos = 0;
    private SampleMecanumDriveCancelable mecanumDrive;
    private int cycle = 0;
    private ElapsedTime TIME = new ElapsedTime();
    private OpenCvCamera camera;
    public double IMU_FREQ = 4;

    @Override
    public void runOpMode() throws InterruptedException {

        mecanumDrive = new SampleMecanumDriveCancelable(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        ControlHub ch = new ControlHub(hardwareMap);
        ControlHub.telemetry = telemetry;
        ExpansionHub eh = new ExpansionHub(hardwareMap, mecanumDrive.getLocalizer());
        Controls c = new Controls(gamepad1, gamepad2);
        Hang g = new Hang();

        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
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

        OutTake outTake = new OutTake(hardwareMap);
        Intake intake = new Intake();
        intake.setPixelStackPosition(0);

        OutTake.finalPivotPivotAngle = 200;
        OutTake.finalArmAngle = 230;
        OutTake.intermediarPivot = 130;
        ExpansionHub.extension_length = 6900;
        ExpansionHub.ImuYawAngle = 0;

        Grippers.FORCE = true;


        TrajectorySequence left = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    OutTake.State.level = 1;
                    readSensor = false;
                    Controls.ExtendElevator = true;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    ExpansionHub.extension_length = 6900;
                    ExpansionHub.ImuYawAngle = 0;
                    OutTake.outTakeExtension.activate();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();

                })
                .lineToLinearHeading(new Pose2d(leftpurple_x, leftpurple_y, leftpurple_h))
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    Controls.ElevatorDown = true;
                    outTake.update_values();
                    outTake.update();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    Controls.DropRight = true;
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    OutTake.outTakeExtension.MOTION_PROFILED = true;
                    OutTake.State.level = 0;
                    ExpansionHub.extension_length = 0;
                    OutTake.outTakeExtension.deactivate();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();
                    Controls.ElevatorUp = true;
                    OutTake.finalPivotPivotAngle = 130;
                    OutTake.finalArmAngle = 210;
                    OutTake.elevatorArm.setArmAngle(OutTake.finalArmAngle);
                    OutTake.elevatorArm.setPivotAngle(OutTake.finalPivotPivotAngle);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    ExpansionHub.extension_length = 6900;
                    OutTake.outTakeExtension.activate();
                    OutTake.outTakeExtension.update();
                    ExpansionHub.ImuYawAngle = -90 + Math.toDegrees(leftyellow_h);
                })
                .lineToLinearHeading(new Pose2d(leftyellow_x, leftyellow_y, leftyellow_h))
                .addTemporalMarker(() -> {
                    Controls.DropLeft = true;
                    OutTake.outTakeExtension.MOTION_PROFILED = false;
                })
                .waitSeconds(0.1)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(transit_x, transit_y, transit_h), -transit_h)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x, transit_y - 10), -transit_h)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(65, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x, transit_y - 48), -transit_h)
                .splineToSplineHeading(new Pose2d(angledStack_x, angledStack_y, angledStack_h), -angledStack_h + 1.2)
                .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> {
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                })
                .addTemporalMarker(() -> {
                    state = State.INTAKE;
                })
                .waitSeconds(0.1)
                .build();

        TrajectorySequence middle = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    OutTake.State.level = 1;
                    readSensor = false;
                    Controls.ExtendElevator = true;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.6, () -> {
                    ExpansionHub.extension_length = 6900;
                    ExpansionHub.ImuYawAngle = 0;
                    OutTake.outTakeExtension.activate();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();

                })
                .lineToLinearHeading(middlePurple)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    Controls.ElevatorDown = true;
                    outTake.update_values();
                    outTake.update();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    Controls.DropRight = true;
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    OutTake.outTakeExtension.MOTION_PROFILED = true;
                    OutTake.State.level = 0;
                    ExpansionHub.extension_length = 0;
                    OutTake.outTakeExtension.deactivate();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();
                    Controls.ElevatorUp = true;
                    OutTake.finalPivotPivotAngle = 130;
                    OutTake.finalArmAngle = 210;
                    OutTake.elevatorArm.setArmAngle(OutTake.finalArmAngle);
                    OutTake.elevatorArm.setPivotAngle(OutTake.finalPivotPivotAngle);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    ExpansionHub.extension_length = 6900;
                    OutTake.outTakeExtension.activate();
                    OutTake.outTakeExtension.update();
                    ExpansionHub.ImuYawAngle = -90 + Math.toDegrees(middleYellow.getHeading());
                })
                .lineToLinearHeading(middleYellow)
                .addTemporalMarker(() -> {
                    Controls.DropLeft = true;
                    OutTake.outTakeExtension.MOTION_PROFILED = true;
                })
                .waitSeconds(0.1)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(transit_x + 1, transit_y, transit_h), -transit_h)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x + 1, transit_y - 10), -transit_h)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(65, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x + 1, transit_y - 48), -transit_h)
                .splineToSplineHeading(new Pose2d(angledStack_x, angledStack_y, angledStack_h), -angledStack_h + 1.2)
                .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> {
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                })
                .addTemporalMarker(() -> {
                    state = State.INTAKE;
                })
                .waitSeconds(0.1)
                .build();

        TrajectorySequence right = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    OutTake.State.level = 1;
                    readSensor = false;
                    Controls.ExtendElevator = true;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    ExpansionHub.extension_length = 6900;
                    ExpansionHub.ImuYawAngle = 0;
                    OutTake.outTakeExtension.activate();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();

                })
                .lineToLinearHeading(new Pose2d(rightpurple_x, rightpurple_y, rightpurple_h))
                .waitSeconds(0.2)
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    Controls.ElevatorDown = true;
                    outTake.update_values();
                    outTake.update();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    Controls.DropRight = true;
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    OutTake.outTakeExtension.MOTION_PROFILED = true;
                    OutTake.State.level = 0;
                    ExpansionHub.extension_length = 0;
                    OutTake.outTakeExtension.deactivate();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();
                    Controls.ElevatorUp = true;
                    OutTake.finalPivotPivotAngle = 130;
                    OutTake.finalArmAngle = 210;
                    OutTake.elevatorArm.setArmAngle(OutTake.finalArmAngle);
                    OutTake.elevatorArm.setPivotAngle(OutTake.finalPivotPivotAngle);
                })
                .UNSTABLE_addTemporalMarkerOffset(1, () -> {
                    ExpansionHub.extension_length = 6900;
                    OutTake.outTakeExtension.activate();
                    OutTake.outTakeExtension.update();
                    ExpansionHub.ImuYawAngle = -90 + Math.toDegrees(leftyellow_h);
                })
                .lineToLinearHeading(new Pose2d(rightyellow_x, rightyellow_y, rightyellow_h))
                .addTemporalMarker(() -> {
                    Controls.DropLeft = true;
                    OutTake.outTakeExtension.MOTION_PROFILED = false;
                })
                .waitSeconds(0.1)
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(transit_x, transit_y, transit_h), -transit_h)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(50, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x, transit_y - 10), -transit_h)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(65, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x, transit_y - 48), -transit_h)
                .splineToSplineHeading(new Pose2d(angledStack_x, angledStack_y, angledStack_h), -angledStack_h + 1.2)
                .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> {
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                })
                .addTemporalMarker(() -> {
                    state = State.INTAKE;
                })
                .waitSeconds(0.1)
                .build();

        TrajectorySequence angledIntake = mecanumDrive.trajectorySequenceBuilder(left.end())
                .addTemporalMarker(() -> {
                    stackPos = min(stackPos + 1, 4);
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                })
                .forward(3)
                .back(3)
                .forward(3)
                .back(3)
                .addTemporalMarker(() -> {
                    stackPos = min(stackPos + 1, 4);
                    intake.servo.setAngle(0);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, -0.65);
                })
                .waitSeconds(0.3)
                .build();

        TrajectorySequence angledToBackDrop = mecanumDrive.trajectorySequenceBuilder(left.end())
                .addTemporalMarker(() -> {
                    intake.servo.setAngle(0);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                })
                .addTemporalMarker(0.3, () -> {
                    intake.servo.setAngle(0);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, -1);
                })
                .addTemporalMarker(1, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                })
                .addTemporalMarker(1.4, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                })
                .lineToSplineHeading(new Pose2d(transit_x, transit_y - 48, transit_h))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(45, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x, transit_y - 30), transit_h)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(65, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x, transit_y), transit_h)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    OutTake.State.level = 5;
                    Controls.ExtendElevator = true;
                    readSensor = true;
                })
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(40))
                .splineToSplineHeading(new Pose2d(leftyellow_x-15, leftyellow_y, Math.toRadians(60)), Math.toRadians(60))
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                    readSensor = false;
                })
                .addTemporalMarker(() -> {
                    if(TIME.seconds() < 22) {
                        if(cycle != 2) mecanumDrive.followTrajectorySequenceAsync(toAngledStack);
                        else mecanumDrive.followTrajectorySequenceAsync(otherToAngledStack);
                    }
                })
                .build();

        TrajectorySequence otherAngledIntake = mecanumDrive.trajectorySequenceBuilder(new Pose2d(left.end().getX() + 12, left.end().getY(), left.end().getHeading()))
                .addTemporalMarker(() -> {
                    stackPos = min(stackPos + 1, 4);
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                })
                .forward(3)
                .back(3)
                .forward(3)
                .back(3)
                .addTemporalMarker(() -> {
                    stackPos = min(stackPos + 1, 4);
                    intake.servo.setAngle(0);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, -0.65);
                })
                .waitSeconds(0.3)
                .build();

        TrajectorySequence otherAngledToBackDrop = mecanumDrive.trajectorySequenceBuilder(otherAngledIntake.end())
                .addTemporalMarker(() -> {
                    intake.servo.setAngle(0);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                })
                .addTemporalMarker(0.3, () -> {
                    intake.servo.setAngle(0);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, -1);
                })
                .addTemporalMarker(1, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                })
                .addTemporalMarker(1.4, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                })
                .lineToSplineHeading(new Pose2d(transit_x - 1, transit_y - 48, transit_h))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(45, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x - 1, transit_y - 30), transit_h)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(65, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x - 1, transit_y), transit_h)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    OutTake.State.level = 5;
                    Controls.ExtendElevator = true;
                    readSensor = true;
                })
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(40))
                .splineToSplineHeading(new Pose2d(leftyellow_x-15, leftyellow_y, Math.toRadians(60)), Math.toRadians(60))
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                    readSensor = false;
                })
                .waitSeconds(0.01)
                .build();

        otherToAngledStack  = mecanumDrive.trajectorySequenceBuilder(angledToBackDrop.end())
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(transit_x, transit_y, transit_h), -transit_h)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(45, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x, transit_y - 10), -transit_h)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(65, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x, transit_y - 48), -transit_h)
                .splineToSplineHeading(new Pose2d(angledStack_x, angledStack_y, angledStack_h), -angledStack_h + 1.2)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    stackPos = 4;
                    intake.servo.setAngle(Intake.stackPositions[4]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                })
                .lineToLinearHeading(new Pose2d(angledStack_x + 12, angledStack_y, angledStack_h))
                .UNSTABLE_addTemporalMarkerOffset(-0.7, () -> {
                    stackPos = 0;
                    intake.servo.setAngle(Intake.stackPositions[0]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                })
                .addTemporalMarker(() -> {
                    state = State.OTHER_INTAKE;
                })
                .waitSeconds(0.1)
                .build();

        toAngledStack = mecanumDrive.trajectorySequenceBuilder(angledToBackDrop.end())
                .addTemporalMarker(()-> {
                    Grippers.FORCE = true;
                })
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(transit_x, transit_y, transit_h))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(45, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x, transit_y - 10), -transit_h)
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(65, 5, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(transit_x, transit_y - 48), -transit_h)
                .splineToSplineHeading(new Pose2d(angledStack_x, angledStack_y, angledStack_h), -angledStack_h + 1.2)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    stackPos = min(stackPos + 2, 4);
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                })
                .addTemporalMarker(() -> {
                    state = State.INTAKE;
                })
                .waitSeconds(0.1)
                .build();

        OutTake.leftGripper.update_values();
        OutTake.rightGripper.update_values();

        OutTake.leftGripper.update();
        OutTake.rightGripper.update();

        while(opModeInInit()){
            outTake.update();
            outTake.update_values();
            if(detector.getLocation() == BlueCloseDetectionPipeline.Location.LEFT)
                telemetry.addLine("LEFT");
            else if(detector.getLocation() == BlueCloseDetectionPipeline.Location.MIDDLE)
                telemetry.addLine("MIDDLE");
            else if(detector.getLocation() == BlueCloseDetectionPipeline.Location.RIGHT)
                telemetry.addLine("RIGHT");
            telemetry.update();
        }

        new Thread(() -> {
            camera.closeCameraDevice();
        }).start();

        TIME.reset();
        if(detector.getLocation() == BlueCloseDetectionPipeline.Location.LEFT)
            mecanumDrive.followTrajectorySequenceAsync(right);
        else if(detector.getLocation() == BlueCloseDetectionPipeline.Location.MIDDLE)
            mecanumDrive.followTrajectorySequenceAsync(middle);
        else if(detector.getLocation() == BlueCloseDetectionPipeline.Location.RIGHT)
            mecanumDrive.followTrajectorySequenceAsync(left);
        freq.reset();

        ElapsedTime imuTime = new ElapsedTime();

        while (opModeIsActive() && !isStopRequested()){

            for(LynxModule m : ControlHub.all){
                m.clearBulkCache();
            }

            if(readSensor){
                ExpansionHub.ImuYawAngle = -90 + Math.toDegrees(mecanumDrive.getPoseEstimate().getHeading());
                if(ExpansionHub.ImuYawAngle > 180) ExpansionHub.ImuYawAngle -= 360;
                if(ExpansionHub.ImuYawAngle < -180) ExpansionHub.ImuYawAngle += 360;
            }

            if(cycle == 2 && state == State.INTAKE) {
                state = State.OTHER_INTAKE;
            }

            if(state == State.INTAKE){
                    if (OutTake.fullPixel() || TIME.seconds() > 26) {
                        state = null;
                        mecanumDrive.breakFollowing();
                        mecanumDrive.followTrajectorySequenceAsync(angledToBackDrop);

                        cycle++;
                    } else if (!mecanumDrive.isBusy()) {
                        mecanumDrive.followTrajectorySequenceAsync(angledIntake);
                    }
            }

            if(state == State.OTHER_INTAKE) {
                if (OutTake.fullPixel() || TIME.seconds() > 26) {
                    state = null;
                    mecanumDrive.breakFollowing();
                    mecanumDrive.followTrajectorySequenceAsync(otherAngledToBackDrop);

                    cycle++;
                } else if (!mecanumDrive.isBusy()) {
                    mecanumDrive.followTrajectorySequenceAsync(otherAngledIntake);
                }
            }

            if(relocalize){


                relocalize = false;
            }

            mecanumDrive.update();

            if(imuTime.seconds() > 1.0 / IMU_FREQ) {
                imuTime.reset();
                double imuAngle = ExpansionHub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                Pose2d pose = mecanumDrive.getPoseEstimate();
                mecanumDrive.setPoseEstimate(new Pose2d(pose.getX(), pose.getY(), imuAngle));
            }

            outTake.update();
            outTake.update_values();
            intake.servo.update();

            outTake.runTelemetry();
            telemetry.addData("angle", ExpansionHub.ImuYawAngle);
            telemetry.addData("freq", 1.0/freq.seconds());

            telemetry.addData("x: ", mecanumDrive.getPoseEstimate().getX());
            telemetry.addData("y: ", mecanumDrive.getPoseEstimate().getY());
            telemetry.addData("h: ", mecanumDrive.getPoseEstimate().getHeading());
            freq.reset();

            telemetry.update();
            c.loop();
        }

        OutTake.intermediarPivot = 130;
        OutTake.finalArmAngle = 210;
    }
    ElapsedTime freq = new ElapsedTime();
}
