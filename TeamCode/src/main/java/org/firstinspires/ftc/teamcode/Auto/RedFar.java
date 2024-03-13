package org.firstinspires.ftc.teamcode.Auto;

import static org.firstinspires.ftc.teamcode.Parts.OutTake.State.step;
import static org.firstinspires.ftc.teamcode.utils.RedFarDetectionPipeline.Location.LEFT;
import static org.firstinspires.ftc.teamcode.utils.RedFarDetectionPipeline.Location.MIDDLE;
import static java.lang.Math.min;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.OutTake;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utils.RedFarDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

// purple
// 38, 11, Math.toRadians(270)
// stack
// 50, 18, Math.toRadians(270)

@Autonomous(name = "redFar")
@Config
public class RedFar extends LinearOpMode {
    enum State{
        INTAKE,
        NOT_INTAKE,
        GET_IN,
        GET_OUT
    }
    enum CASE {
        LEFT,
        MIDDLE,
        RIGHT
    }

    private CASE caz = CASE.LEFT;

    private State state = State.NOT_INTAKE;
    private boolean relocalize = false;
    private int cycle = 0;
    private ElapsedTime get_out_timer = new ElapsedTime();
    private boolean align = true;

    public static double
            stack_x = 50, stack_y = 18.5, stack_h = Math.toRadians(-90),
            park_x = 49, park_y = -92, park_h = Math.toRadians(-90),
            middlepurple_x = 37, middlepurple_y = 14.5, middlepurple_h = Math.toRadians(-90),
            leftpurple_x = 10.5, leftpurple_y = 7, leftpurple_h = Math.toRadians(0),
            rightpurple_x = 16, rightpurple_y = -3, rightpurple_h = Math.toRadians(300),
    middleyellow_x = 31.5, middleyellow_y = -87, middleyellow_h = Math.toRadians(-110);

    boolean readSensor = false;

    private int stackPos = 0;
    private SampleMecanumDriveCancelable mecanumDrive;
    private ElapsedTime TIME = new ElapsedTime();
    private OpenCvCamera camera;
    public double IMU_FREQ = 4;

    public TrajectorySequence left, middle, right;
    double TRAJ_TIME = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        mecanumDrive = new SampleMecanumDriveCancelable(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        ControlHub ch = new ControlHub(hardwareMap);
        ControlHub.telemetry = telemetry;
        ExpansionHub eh = new ExpansionHub(hardwareMap, mecanumDrive.getLocalizer());
        Controls c = new Controls(gamepad1, gamepad2);

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

        OutTake outTake = new OutTake(hardwareMap);
        Intake intake = new Intake();
        intake.setPixelStackPosition(0);

        OutTake.leftGripper.update_values();
        OutTake.rightGripper.update_values();

        OutTake.leftGripper.update();
        OutTake.rightGripper.update();

        OutTake.finalPivotPivotAngle = 200;
        OutTake.finalArmAngle = 230;
        OutTake.intermediarPivot = 130;
        ExpansionHub.extension_length = 6900;
        ExpansionHub.ImuYawAngle = 0;

        middle = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .lineToLinearHeading(new Pose2d(middlepurple_x, middlepurple_y, middlepurple_h))
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    OutTake.State.level = 1;
                    Controls.ExtendElevator = true;
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.8, () -> {
                                ExpansionHub.extension_length = 220;
                                ExpansionHub.ImuYawAngle = 0;
                                OutTake.outTakeExtension.activate();
                                OutTake.outTakeExtension.update();
                                OutTake.outTakeExtension.update_values();
                                OutTake.outTakeExtension.update();
                                OutTake.outTakeExtension.update_values();
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.2, () -> {
                    OutTake.elevator.setInstantPosition(OutTake.State.level * step - step / 2);
                    OutTake.State.level--;
                    outTake.update_values();
                    outTake.update();
                    outTake.update_values();
                    outTake.update();
                })
                .waitSeconds(0.01)
                .addTemporalMarker(() -> {
                    Controls.DropLeft = true;
                })
                .waitSeconds(0.6)
                .addTemporalMarker(() -> {
                    OutTake.finalPivotPivotAngle = 130;
                    OutTake.finalArmAngle = 210;
                })
                .lineToLinearHeading(new Pose2d(stack_x, stack_y, stack_h))
                .UNSTABLE_addTemporalMarkerOffset(-0.15, () -> {
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                    state = State.INTAKE;
                })
                .build();

        left = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    OutTake.State.level = 1;
                    Controls.ExtendElevator = true;
                })
                .addTemporalMarker(0.7, () -> {
                    ExpansionHub.extension_length = 6900;
                    align = false;
                    ExpansionHub.ImuYawAngle = 0;
                    OutTake.outTakeExtension.activate();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();
                })
                .lineToLinearHeading(new Pose2d(leftpurple_x, leftpurple_y, leftpurple_h))
                .addTemporalMarker(() -> {
                    OutTake.elevator.setInstantPosition(0);
                    OutTake.State.level--;
                    outTake.update_values();
                    outTake.update();
                    outTake.update_values();
                    outTake.update();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    Controls.DropLeft = true;
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    align = true;
                    OutTake.finalPivotPivotAngle = 130;
                    OutTake.finalArmAngle = 210;
                })
                .lineToLinearHeading(new Pose2d(leftpurple_x, -6, leftpurple_h))
                .lineToLinearHeading(new Pose2d(stack_x - 5, 0, 0))
                .lineToLinearHeading(new Pose2d(stack_x, stack_y, stack_h))
                .UNSTABLE_addTemporalMarkerOffset(-0.15, () -> {
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                    state = State.INTAKE;
                })
                .build();

        right = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    OutTake.State.level = 1;
                    Controls.ExtendElevator = true;
                })
                .addTemporalMarker(0.7, () -> {
                    ExpansionHub.extension_length = 6900;
                    align = false;
                    ExpansionHub.ImuYawAngle = 0;
                    OutTake.outTakeExtension.activate();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();
                })
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(65, 2, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(30))
                .splineToSplineHeading(new Pose2d(rightpurple_x, rightpurple_y, rightpurple_h), rightpurple_h)
                .addTemporalMarker(() -> {
                    OutTake.elevator.setInstantPosition(0);
                    OutTake.State.level--;
                    outTake.update_values();
                    outTake.update();
                    outTake.update_values();
                    outTake.update();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    Controls.DropLeft = true;
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    align = true;
                    OutTake.finalPivotPivotAngle = 130;
                    OutTake.finalArmAngle = 210;
                })
                .lineToLinearHeading(new Pose2d(stack_x, stack_y - 3, stack_h))
                .addTemporalMarker(() -> {
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                    state = State.INTAKE;
                })
                .lineToLinearHeading(new Pose2d(stack_x, stack_y, stack_h))
                .build();

        TrajectorySequence preload = mecanumDrive.trajectorySequenceBuilder(middle.end())
                .addTemporalMarker(() -> {
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
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, -0.8);
                })
                .waitSeconds(0.3)
                .build();

        TrajectorySequence intakein = mecanumDrive.trajectorySequenceBuilder(middle.end())
                .addTemporalMarker(() -> {
                    if(cycle == 2) stackPos = 4;
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
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, -0.8);
                })
                .waitSeconds(0.3)
                .build();

        TrajectorySequence yellowMiddle = mecanumDrive.trajectorySequenceBuilder(new Pose2d(stack_x, stack_y, stack_h))
                .splineToConstantHeading(new Vector2d(stack_x, stack_y - 80), stack_h)
                .splineToSplineHeading(new Pose2d(middleyellow_x+5, middleyellow_y + 0.4, middleyellow_h), middleyellow_h - 0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> {
                    Controls.DropLeft = true;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.2, () -> {
                    OutTake.outTakeExtension.deactivate();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    OutTake.outTakeExtension.activate();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                })
                .lineToLinearHeading(new Pose2d(middleyellow_x, middleyellow_y, middleyellow_h))
                .UNSTABLE_addTemporalMarkerOffset(-1.8, () -> {
                    OutTake.State.level = 2;
                    ExpansionHub.extension_length = 6900;
                    Controls.ExtendElevator = true;
                })
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    OutTake.elevator.setInstantPosition(OutTake.State.level * step - step / 2);
                })
                .waitSeconds(0.1)
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(stack_x, stack_y - 80, stack_h))
                .splineToConstantHeading(new Vector2d(stack_x, stack_y - 79.9), -stack_h)
                .splineToConstantHeading(new Vector2d(stack_x, stack_y), -stack_h)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    stackPos = min(stackPos + 1, 4);
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                    state = State.INTAKE;
                })
                .build();

        TrajectorySequence yellowRight = mecanumDrive.trajectorySequenceBuilder(new Pose2d(stack_x, stack_y, stack_h))
                .splineToConstantHeading(new Vector2d(stack_x, stack_y - 80), stack_h)
                .splineToSplineHeading(new Pose2d(middleyellow_x-1, middleyellow_y, middleyellow_h), middleyellow_h - 0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> {
                    Controls.DropLeft = true;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    OutTake.outTakeExtension.deactivate();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    OutTake.outTakeExtension.activate();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                })
                .lineToLinearHeading(new Pose2d(middleyellow_x - 8, middleyellow_y, middleyellow_h))
                .UNSTABLE_addTemporalMarkerOffset(-1.8, () -> {
                    OutTake.State.level = 2;
                    ExpansionHub.extension_length = 6900;
                    Controls.ExtendElevator = true;
                })
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    OutTake.elevator.setInstantPosition(OutTake.State.level * step - step / 2);
                })
                .waitSeconds(0.1)
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(stack_x, stack_y - 80, stack_h))
                .splineToConstantHeading(new Vector2d(stack_x, stack_y - 79.9), -stack_h)
                .splineToConstantHeading(new Vector2d(stack_x, stack_y), -stack_h)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    stackPos = min(stackPos + 1, 4);
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                    state = State.INTAKE;
                })
                .build();

        TrajectorySequence yellowLeft = mecanumDrive.trajectorySequenceBuilder(new Pose2d(stack_x, stack_y, stack_h))
                .splineToConstantHeading(new Vector2d(stack_x, stack_y - 80), stack_h)
                .splineToSplineHeading(new Pose2d(middleyellow_x+6, middleyellow_y, middleyellow_h), middleyellow_h - 0.5)
                .UNSTABLE_addTemporalMarkerOffset(0.01, () -> {
                    Controls.DropRight = true;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    OutTake.elevator.setInstantPosition(OutTake.State.level * step - step / 2);
                    OutTake.outTakeExtension.deactivate();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                })
                .UNSTABLE_addTemporalMarkerOffset(0.3, () -> {
                    OutTake.outTakeExtension.activate();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                })
                .waitSeconds(0.1)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    OutTake.elevator.setInstantPosition(3 * step);
                })
                .lineToLinearHeading(new Pose2d(middleyellow_x - 3, middleyellow_y + 0.5, middleyellow_h))
                .UNSTABLE_addTemporalMarkerOffset(-1.8, () -> {
                    OutTake.State.level = 2;
                    ExpansionHub.extension_length = 6900;
                    Controls.ExtendElevator = true;
                })
                .addTemporalMarker(() -> {
                    Controls.DropLeft = true;
                })
                .waitSeconds(0.1)
                .setReversed(true)
                .lineToSplineHeading(new Pose2d(stack_x, stack_y - 80, stack_h))
                .splineToConstantHeading(new Vector2d(stack_x, stack_y - 79.9), -stack_h)
                .splineToConstantHeading(new Vector2d(stack_x, stack_y), -stack_h)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    stackPos = min(stackPos + 1, 4);
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                    state = State.INTAKE;
                })
                .build();

        TrajectorySequence toBackDrop = mecanumDrive.trajectorySequenceBuilder(new Pose2d(stack_x, stack_y, stack_h))
                .splineToConstantHeading(new Vector2d(stack_x, stack_y - 80), stack_h)
                .splineToSplineHeading(new Pose2d(stack_x - 13, middleyellow_y - 1, Math.toRadians(-120)), Math.toRadians(-120))
                .UNSTABLE_addTemporalMarkerOffset(-1.8, () -> {
                    OutTake.State.level = 3;
                    ExpansionHub.extension_length = 6900;
                    Controls.ExtendElevator = true;
                })
                .UNSTABLE_addTemporalMarkerOffset(-0.1, () -> {
                    Controls.DropLeft = true;
                    Controls.DropRight = true;
                })
                .addTemporalMarker(() -> {
                    if(cycle == 3 || TIME.seconds() > 23) {
                        mecanumDrive.breakFollowing();
                        mecanumDrive.followTrajectorySequenceAsync(mecanumDrive.trajectorySequenceBuilder(mecanumDrive.getPoseEstimate()).lineToLinearHeading(new Pose2d(park_x, park_y, park_h)).build());
                        TRAJ_TIME = TIME.seconds();
                    }
                })
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(60, 5, DriveConstants.TRACK_WIDTH))
                .setReversed(true)
                .splineToSplineHeading(new Pose2d(stack_x, stack_y - 80, stack_h), -stack_h)
                .splineToConstantHeading(new Vector2d(stack_x, stack_y - 70), -stack_h)
                .splineToConstantHeading(new Vector2d(stack_x, stack_y), -stack_h)
                .UNSTABLE_addTemporalMarkerOffset(-1, () -> {
                    stackPos = min(stackPos + 1, 4);
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                    state = State.INTAKE;
                }).waitSeconds(0.3)
                .build();

        OutTake.leftGripper.update_values();
        OutTake.rightGripper.update_values();

        OutTake.leftGripper.update();
        OutTake.rightGripper.update();

        while(opModeInInit()){
            outTake.update();
            outTake.update_values();
            if(detector.getLocation() == LEFT)
                telemetry.addLine("LEFT");
            else if(detector.getLocation() == RedFarDetectionPipeline.Location.MIDDLE)
                telemetry.addLine("MIDDLE");
            else if(detector.getLocation() == RedFarDetectionPipeline.Location.RIGHT)
                telemetry.addLine("RIGHT");
            telemetry.update();
        }

        new Thread(() -> {
            camera.closeCameraDevice();
        }).start();

        TIME.reset();
        if(detector.getLocation() == LEFT)
            mecanumDrive.followTrajectorySequenceAsync(left);
        else if(detector.getLocation() == RedFarDetectionPipeline.Location.MIDDLE)
            mecanumDrive.followTrajectorySequenceAsync(middle);
        else if(detector.getLocation() == RedFarDetectionPipeline.Location.RIGHT)
            mecanumDrive.followTrajectorySequenceAsync(right);
        freq.reset();

        ElapsedTime imuTime = new ElapsedTime();

        while (opModeIsActive() && !isStopRequested()){

            for(LynxModule m : ControlHub.all){
                m.clearBulkCache();
            }

            mecanumDrive.update();
            if(align) ExpansionHub.ImuYawAngle = 90 + Math.toDegrees(mecanumDrive.getPoseEstimate().getHeading());

            if(imuTime.seconds() > 1.0 / IMU_FREQ) {
                imuTime.reset();
                double imuAngle = ExpansionHub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                Pose2d pose = mecanumDrive.getPoseEstimate();
                mecanumDrive.setPoseEstimate(new Pose2d(pose.getX(), pose.getY(), imuAngle));
            }

            if(state == State.GET_IN) {
                if(!OutTake.fullPixel()) get_out_timer.reset();
                if(OutTake.fullPixel() && get_out_timer.seconds() > 0.4 ) {
                    state = State.GET_OUT;
                    intake.servo.setAngle(60);
                    get_out_timer.reset();
                } else {
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                }

            } else if(state == State.GET_OUT) {
                if(get_out_timer.seconds() > 0.6) {
                    state = State.NOT_INTAKE;
                    intake.servo.setAngle(60);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                } else {
                    intake.servo.setAngle(60);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, -1);
                }
            }

            if(state == State.INTAKE) {

                if(OutTake.fullPixel() || TIME.seconds() > 25) {
                    state = State.GET_IN;
                    get_out_timer.reset();
                    mecanumDrive.breakFollowing();
                    if(cycle == 0) {
                        if(detector.getLocation() == LEFT)
                            mecanumDrive.followTrajectorySequenceAsync(yellowLeft);
                        else if(detector.getLocation() == MIDDLE)
                            mecanumDrive.followTrajectorySequenceAsync(yellowMiddle);
                        else
                            mecanumDrive.followTrajectorySequenceAsync(yellowRight);
                    }
                    else
                        mecanumDrive.followTrajectorySequenceAsync(toBackDrop);

                    cycle++;
                } else if(!mecanumDrive.isBusy()) {
                    if(cycle == 0)
                        mecanumDrive.followTrajectorySequenceAsync(preload);
                    else
                        mecanumDrive.followTrajectorySequenceAsync(intakein);
                }

            }

            outTake.update();
            outTake.update_values();
            intake.servo.update();

            freq.reset();

            telemetry.update();
            c.loop();
        }

        OutTake.intermediarPivot = 130;
        OutTake.finalArmAngle = 210;
    }
    ElapsedTime freq = new ElapsedTime();
}
