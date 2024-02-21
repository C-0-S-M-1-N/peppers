package org.firstinspires.ftc.teamcode.Auto;

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
    middleYellow_x = 25.24, middleYellow_y = -23.56,
    stack_x = 21, stack_y = 76.5,
    prebackdropMiddle_x = 25.24, prebackdropMiddle_y = -6;

    public static Pose2d middlePurple = new Pose2d(middlePurple_x, middlePurple_y, 0),
    middleYellow = new Pose2d(middleYellow_x, middleYellow_y, Math.toRadians(-90)),
    stack = new Pose2d(stack_x, stack_y, Math.toRadians(-90)),
    prebackdropMiddle = new Pose2d(prebackdropMiddle_x, prebackdropMiddle_y, Math.toRadians(-89));
    public DistanceSensor distanceSensor;
    boolean readSensor = false;
    private TrajectorySequence takeFromStack, toBackdrop;

    private int stackPos = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        ControlHub ch = new ControlHub(hardwareMap);
        ControlHub.telemetry = telemetry;
        ExpansionHub eh = new ExpansionHub(hardwareMap);
        Controls c = new Controls(gamepad1, gamepad2);
        distanceSensor = hardwareMap.get(DistanceSensor.class, "sensor");

        OutTake outTake = new OutTake(hardwareMap);
        Intake intake = new Intake();
        intake.setPixelStackPosition(0);

        OutTake.finalPivotPivotAngle = 200;
        OutTake.finalArmAngle = 230;
        OutTake.intermediarPivot = 130;
        ExpansionHub.sensorDistance = 260;



        SampleMecanumDriveCancelable mecanumDrive = new SampleMecanumDriveCancelable(hardwareMap);


        TrajectorySequence middle = mecanumDrive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    OutTake.State.level = 0;
                    Controls.ExtendElevator = true;
                })
                .UNSTABLE_addTemporalMarkerOffset(0.4, () -> {
                    OutTake.outTakeExtension.activate();
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
                .UNSTABLE_addTemporalMarkerOffset(-0.6, () -> {
                    intake.servo.setAngle(Intake.stackPositions[stackPos]);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 1);
                })
                .addTemporalMarker(() -> {
                    state = State.INTAKE;
                })
                .build();


        takeFromStack = mecanumDrive.trajectorySequenceBuilder(middle.end())
                .addTemporalMarker(() -> {
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
                    intake.servo.setAngle(0);
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, -1);
                })
                .addTemporalMarker(0.4, () -> {
                    ControlHub.setMotorPower(MOTOR_PORTS.M2, 0);
                })
                .lineToLinearHeading(prebackdropMiddle)
                .UNSTABLE_addTemporalMarkerOffset(-0.3, () -> {
                    OutTake.State.level ++;
                    Controls.ExtendElevator = true;
                    relocalize = true;
                })
                .splineToConstantHeading(new Vector2d(middleYellow_x, middleYellow_y), Math.toRadians(-89))
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                })
                .waitSeconds(0.3)
                .lineToLinearHeading(stack)
                .addTemporalMarker(() -> {
                    state = State.INTAKE;
                    mecanumDrive.followTrajectorySequenceAsync(takeFromStack);
                })
                .waitSeconds(0)
                .build();

        OutTake.leftGripper.update_values();
        OutTake.rightGripper.update_values();

        OutTake.leftGripper.update();
        OutTake.rightGripper.update();

        while(opModeInInit()){
            outTake.update();
            outTake.update_values();
        }
        mecanumDrive.followTrajectorySequenceAsync(middle);
        freq.reset();


        while (opModeIsActive() && !isStopRequested()){

            if(readSensor){
                ExpansionHub.sensorDistance = distanceSensor.getDistance(DistanceUnit.MM);
                ExpansionHub.ImuYawAngle = 90 + Math.toDegrees(mecanumDrive.getPoseEstimate().getHeading());
                if(ExpansionHub.ImuYawAngle > 180) ExpansionHub.ImuYawAngle -= 360;
                if(ExpansionHub.ImuYawAngle < -180) ExpansionHub.ImuYawAngle += 360;
            }

            if(state == State.INTAKE){
                if(OutTake.fullPixel()){
                    state = null;
                    mecanumDrive.breakFollowing();
                    mecanumDrive.followTrajectorySequenceAsync(toBackdrop);
                    stackPos ++;
                } else if(!mecanumDrive.isBusy()){
                    stackPos ++;
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
            freq.reset();

            telemetry.update();
            c.loop();
        }

        OutTake.intermediarPivot = 130;
        OutTake.finalArmAngle = 210;
    }
    ElapsedTime freq = new ElapsedTime();
}
