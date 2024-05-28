package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Components.ElevatorArm;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.OutTakeMTI;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;

@Autonomous(name = "RedCloseFar", group = "autos")
public class RedCloseFar extends LinearOpMode {

    public static Pose2d
    MiddlePurple = new Pose2d(0, 0, 0),
    MiddleYellow = new Pose2d(0, 0, 0),

    BackDropGate = new Pose2d(0, 0, 0),
    StackGate = new Pose2d(0, 0, 0),
    Stack = new Pose2d(0, 0, 0),
    Stack2 = new Pose2d(0, 0, 0),
    Backdrop = new Pose2d(0, 0, 0);

    SampleMecanumDriveCancelable drive;
    OutTakeMTI outTake;
    Intake intake;
    int intakeActive = 0, pixelsInStack = 5, order = 0;
    private boolean ack = false, pixelsUpdated = false, isInPreloadPhase = false, firstPathAfterPreload = false;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub e = new ExpansionHub(hardwareMap, drive.getLocalizer());
        Controls cn = new Controls(gamepad1, gamepad2);

        OutTakeMTI.timeToDrop = 0.15;
        OutTakeMTI.extendingAngle = 100;

        TrajectorySequence middle = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    outTake.setToPurplePlacing();
                    isInPreloadPhase = true;
                })
                .lineToLinearHeading(MiddlePurple)
                .waitSeconds(0.1)
                .addTemporalMarker(() -> {
                    Controls.DropLeft = true;
                    Controls.DropLeftAck = true;
                })
                .waitSeconds(0.2)
                .addTemporalMarker(() -> {
                    OutTakeMTI.State.level = 1;
                    OutTakeMTI.elevator.setTargetPosition(3.5 * OutTakeMTI.STEP);
                    OutTakeMTI.align = true;
                    OutTakeMTI.arm.rotationIndex = 4;
                    outTake.setToNormalPlacingFromPurplePixelPlacing();
                })
                .lineToLinearHeading(MiddleYellow)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropRightAck = true;
                    OutTakeMTI.driverUpdated = true;
                    isInPreloadPhase = false;
                    firstPathAfterPreload = true;
                    OutTakeMTI.arm.rotationIndex = 4;

                })
                .build();

        TrajectorySequence goToStackFromPreloadM = drive.trajectorySequenceBuilder(middle.end())
                .setReversed(true)
                .addTemporalMarker(() -> {
                    firstPathAfterPreload = false;
                })

                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(55, 3.1415, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .splineToSplineHeading(BackDropGate, Math.toRadians(90))

                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(65, 3.1415, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .splineToSplineHeading(StackGate, Math.toRadians(0))
                .addTemporalMarker(() -> {
                    intake.setPixelStackPosition(pixelsInStack);
                    intakeActive = 1;
                    pixelsUpdated = false;
                })

                .splineToSplineHeading(Stack, Math.toRadians(0))

                .build();

        TrajectorySequence takeFromStack = drive.trajectorySequenceBuilder(Stack)
                .addTemporalMarker(() -> {
                    intakeActive = 1;
                    pixelsUpdated = false;
                    intake.setPixelStackPosition(pixelsInStack);
                })
                .forward(2)
                .back(2)
                .forward(2)
                .back(2)
                .addTemporalMarker(() -> {
                    intakeActive = -1;
                    if(!pixelsUpdated) pixelsInStack --;
                })

                .build();

        TrajectorySequence goToBackdrop = drive.trajectorySequenceBuilder(Stack)
                .addTemporalMarker(() -> {
                    intakeActive = -1;
                })
                .setReversed(true)
                .splineToSplineHeading(BackDropGate, 0)
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(55, 4, DriveConstants.TRACK_WIDTH))
                .UNSTABLE_addTemporalMarkerOffset(-0.4, () -> {
                    Controls.ExtendElevator = true;
                    Controls.ExtendElevatorAck = false;
                })
                .splineToSplineHeading(Backdrop, Math.toRadians(-45))
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropLeft = true;
                    Controls.DropLeftAck = false;
                    Controls.DropRightAck = false;
                })
                .waitSeconds(0.2)

                .build();


        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(goToBackdrop.end())
                .setReversed(true)
                .addTemporalMarker(() -> {
                    firstPathAfterPreload = false;
                })

                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(55, 3.1415, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .splineToSplineHeading(BackDropGate, Math.toRadians(90))

                .setVelConstraint(SampleMecanumDriveCancelable.getVelocityConstraint(65, 3.1415, DriveConstants.TRACK_WIDTH))
                .setAccelConstraint(SampleMecanumDriveCancelable.getAccelerationConstraint(50))
                .splineToSplineHeading(StackGate, Math.toRadians(0))
                .addTemporalMarker(() -> {
                    intake.setPixelStackPosition(pixelsInStack);
                    intakeActive = 1;
                    pixelsUpdated = false;
                })

                .splineToSplineHeading(Stack, Math.toRadians(0))

                .build();


        while (opModeInInit()){
            outTake.update();
        }
        long time1 = 0;
        ElapsedTime autoTime = new ElapsedTime();

        while (opModeIsActive()){
            ControlHub.ControlHubModule.clearBulkCache();
            ExpansionHub.ExpansionHubModule.clearBulkCache();
//            e.update(false, drive.getLocalizer().getPoseEstimate().getX(), drive.getLocalizer().getPoseEstimate().getY());
            ExpansionHub.ImuYawAngle = Math.toDegrees(drive.getPoseEstimate().getHeading()) - ExpansionHub.beforeReset;
            if((System.currentTimeMillis() - time1) >= 1.0 / 4){
                double Yawn = ExpansionHub.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
                drive.setPoseEstimate(new Pose2d(drive.getPoseEstimate().getX(), drive.getPoseEstimate().getY(), Yawn));
//                ExpansionHub.ImuYawAngle = Yawn - ExpansionHub.beforeReset;
                time1 = System.currentTimeMillis();
            }
            if((order == 1 && OutTakeMTI.isFullOfPixels())){
                pixelsInStack --;
                if(pixelsInStack < 0) pixelsInStack = 0;
                drive.breakFollowing();
                order ++;
                pixelsUpdated = true;
            }
            else if(order == 1 && OutTakeMTI.hasAPixel() && !ack){
                pixelsInStack --;
                drive.breakFollowing();
                ack = true;
                pixelsUpdated = true;
            }
            Controls.Intake = false;
            Controls.RevIntake = false;

            if(intakeActive == 1){
                Controls.Intake = true;
            } else if(intakeActive == -1){
                Controls.RevIntake = true;
                Intake.forceOut = true;
            }

            if(!isInPreloadPhase && !drive.isBusy()){
                switch (order) {
                    case 0:
                        if(autoTime.seconds() <= 25) {
                            if(firstPathAfterPreload)
                                drive.followTrajectorySequenceAsync(goToStackFromPreloadM);
                            else
                                drive.followTrajectorySequenceAsync(goToStack);
                            order++;
                        }
                        break;
                    case 1:
                        drive.followTrajectorySequenceAsync(takeFromStack);
                        break;
                    case 2:
                        drive.followTrajectorySequenceAsync(goToBackdrop);
                        order = 0;
                    default:
                        break;
                }
            }

            drive.update();

            intake.update_values();
            intake.update();
            outTake.update();
            cn.loop();
            ControlHub.telemetry.addData("pixels in stack", pixelsInStack);
            ControlHub.telemetry.update();
        }
        OutTakeMTI.timeToDrop = 0.3;
        Intake.reversePower = -1;

    }
}
