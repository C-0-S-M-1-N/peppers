package org.firstinspires.ftc.teamcode.Auto;

import static java.lang.Math.E;
import static java.lang.Math.min;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Components.OutTakeExtension;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.OutTake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.internals.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.Hubs;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;
import org.firstinspires.ftc.teamcode.internals.SERVO_PORTS;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utils.AutoServo;


/* x, y
* middle p - (18.5, 0.3)
* middle y - (25.24, -22.56)
* stack - (23.35, 76.01)
*
* */

@Autonomous(name = "redClose")
@Config
public class RedClose extends LinearOpMode {
    enum State{
        INTAKE,
        NOT_INTAKE
    }
    private State state = State.NOT_INTAKE;
    private boolean relocalize = false;

    public static double middlePurple_x = 21.5, middlePurple_y = -1.3,
    middleYellow_x = 25.24, middleYellow_y = -24.5,
    stack_x = 22, stack_y = 76.5,
    prebackdropMiddle_x = 25.24, prebackdropMiddle_y = -6,
    leftpurple_x = 14.5, leftpurple_y = 7, leftpurple_h = 0.44,
    leftyellow_x = 22, leftyellow_y = -24, leftyellow_h = Math.toRadians(-70),
    transit_x = 2, transit_y = 2, transit_h = Math.toRadians(-90);

    public static Pose2d middlePurple = new Pose2d(middlePurple_x, middlePurple_y, 0),
    middleYellow = new Pose2d(middleYellow_x, middleYellow_y, Math.toRadians(-90)),
    stack = new Pose2d(stack_x, stack_y, Math.toRadians(-90)),
    prebackdropMiddle = new Pose2d(prebackdropMiddle_x, prebackdropMiddle_y, Math.toRadians(-89));
    public DistanceSensor distanceSensor;
    boolean readSensor = false;
    private TrajectorySequence takeFromStack, toBackdrop;

    private int stackPos = 0;
    private SampleMecanumDriveCancelable mecanumDrive;
    private int cycle = 0;
    private ElapsedTime TIME = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        mecanumDrive = new SampleMecanumDriveCancelable(hardwareMap);
        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        ControlHub ch = new ControlHub(hardwareMap);
        ControlHub.telemetry = telemetry;
        ExpansionHub eh = new ExpansionHub(hardwareMap, mecanumDrive.getLocalizer());
        Controls c = new Controls(gamepad1, gamepad2);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor");

        OutTake outTake = new OutTake(hardwareMap);
        Intake intake = new Intake();
        intake.setPixelStackPosition(0);

        OutTake.finalPivotPivotAngle = 200;
        OutTake.finalArmAngle = 230;
        OutTake.intermediarPivot = 130;
        ExpansionHub.sensorDistance = 279;
        ExpansionHub.ImuYawAngle = 0;

        TrajectorySequence left = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    OutTake.State.level = 0;
                    readSensor = false;
                    Controls.ExtendElevator = true;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    ExpansionHub.sensorDistance = 279;
                    ExpansionHub.ImuYawAngle = 0;
                    OutTake.outTakeExtension.activate();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();
                })
                .lineToLinearHeading(new Pose2d(leftpurple_x, leftpurple_y, leftpurple_h))
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    Controls.DropLeft = true;
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    OutTake.State.level = 0;
                    ExpansionHub.sensorDistance = 0;
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.deactivate();
                    Controls.ElevatorUp = true;
                    OutTake.finalPivotPivotAngle = 130;
                    OutTake.finalArmAngle = 210;
                    OutTake.elevatorArm.setArmAngle(OutTake.finalArmAngle);
                    OutTake.elevatorArm.setPivotAngle(OutTake.finalPivotPivotAngle);
                })
                .lineToLinearHeading(new Pose2d(leftyellow_x, leftyellow_y, leftyellow_h))
                .addTemporalMarker(() -> {
                    readSensor = true;
                    OutTake.outTakeExtension.activate();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    readSensor = false;
                })
                .waitSeconds(0.1)
                .splineToSplineHeading(new Pose2d(transit_x, transit_y, transit_h), -transit_h)
                .splineToSplineHeading(new Pose2d(transit_x, transit_y + 1, transit_h), -transit_h)
                .splineToSplineHeading(new Pose2d(transit_x, transit_y + 48, transit_h), -transit_h)
                .build();

        TrajectorySequence middle = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    OutTake.State.level = 0;
                    Controls.ExtendElevator = true;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.7, () -> {
                    OutTake.outTakeExtension.activate();
                    OutTake.outTakeExtension.activate();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.update_values();
                })
                .lineToLinearHeading(middlePurple)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                    Controls.DropLeft = true;
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    OutTake.State.level = 0;
                    ExpansionHub.sensorDistance = 0;
                    OutTake.outTakeExtension.update_values();
                    OutTake.outTakeExtension.update();
                    OutTake.outTakeExtension.deactivate();
                    Controls.ElevatorUp = true;
                    OutTake.finalPivotPivotAngle = 130;
                    OutTake.finalArmAngle = 210;
                    OutTake.elevatorArm.setArmAngle(OutTake.finalArmAngle);
                    OutTake.elevatorArm.setPivotAngle(OutTake.finalPivotPivotAngle);
                })
                .lineToLinearHeading(middleYellow)
                .addTemporalMarker(() -> {
                    readSensor = true;
                    OutTake.outTakeExtension.activate();
                })
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    readSensor = false;
                })
                .waitSeconds(0.1)
                .lineToLinearHeading(stack)
                .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> {
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                })
                .addTemporalMarker(() -> {
                    state = State.INTAKE;
                })
                .build();


        takeFromStack = mecanumDrive.trajectorySequenceBuilder(middle.end())
                .addTemporalMarker(() -> {
                    stackPos = min(stackPos + 1, 4);
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                })
                .waitSeconds(1.5)
                .addTemporalMarker(() -> {
                    intake.servo.setAngle(0);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, -1);
                })
                .forward(3)
                .UNSTABLE_addTemporalMarkerOffset(0.1, () -> {
                        intake.servo.setAngle(Intake.stackPositions[stackPos]);
                        ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                })
                .back(3)
                .build();

        toBackdrop = mecanumDrive.trajectorySequenceBuilder(middle.end())
                .addTemporalMarker(0.1, () -> {
                    stackPos += 1;
                    intake.servo.setAngle(0);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, -1);
                })
                .addTemporalMarker(0.4, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                })
                .lineToSplineHeading(prebackdropMiddle)
                .UNSTABLE_addTemporalMarkerOffset(-1.5, () -> {
                    OutTake.State.level = 5;
                    Controls.ExtendElevator = true;
                    relocalize = true;
                })
                .splineToConstantHeading(new Vector2d(middleYellow_x, middleYellow_y), Math.toRadians(-89))
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                })
                .lineToLinearHeading(stack)
                .UNSTABLE_addTemporalMarkerOffset(-1.2, () -> {
                    stackPos = min(stackPos + 1, 4);
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                })
                .addTemporalMarker(() -> {
                    state = State.INTAKE;
                    cycle++;
                    mecanumDrive.followTrajectorySequenceAsync(takeFromStack);
                })
                .build();

        OutTake.leftGripper.update_values();
        OutTake.rightGripper.update_values();

        OutTake.leftGripper.update();
        OutTake.rightGripper.update();

        while(opModeInInit()){
            outTake.update();
            outTake.update_values();
        }
        TIME.reset();
        mecanumDrive.followTrajectorySequenceAsync(left);
        freq.reset();


        while (opModeIsActive() && !isStopRequested()){

            if(readSensor){
                ExpansionHub.sensorDistance = distanceSensor.getDistance(DistanceUnit.MM) + ExpansionHub.VELOCITY_COMPENSATION * mecanumDrive.getLocalizer().getPoseVelocity().getY() + 10;
                ExpansionHub.ImuYawAngle = 90 + Math.toDegrees(mecanumDrive.getPoseEstimate().getHeading());
                if(ExpansionHub.ImuYawAngle > 180) ExpansionHub.ImuYawAngle -= 360;
                if(ExpansionHub.ImuYawAngle < -180) ExpansionHub.ImuYawAngle += 360;
            }

            if(state == State.INTAKE){
                if(OutTake.fullPixel() || TIME.seconds() > 26){
                    state = null;
                    mecanumDrive.breakFollowing();
                    mecanumDrive.followTrajectorySequenceAsync(toBackdrop);
                } else if(!mecanumDrive.isBusy()){
                    mecanumDrive.followTrajectorySequenceAsync(takeFromStack);
                }
            }

            if(relocalize){


                relocalize = false;
            }

            mecanumDrive.update();
            outTake.update();
            outTake.update_values();
            intake.servo.update();

            outTake.runTelemetry();
            telemetry.addData("angle", ExpansionHub.ImuYawAngle);
            telemetry.addData("freq", 1.0/freq.seconds());

            telemetry.addData("x: ", mecanumDrive.getPoseEstimate().getX());
            telemetry.addData("y: ", mecanumDrive.getPoseEstimate().getX());
            telemetry.addData("h: ", mecanumDrive.getPoseEstimate().getX());
            freq.reset();

            telemetry.update();
            c.loop();
        }

        OutTake.intermediarPivot = 130;
        OutTake.finalArmAngle = 210;
    }
    ElapsedTime freq = new ElapsedTime();
}
