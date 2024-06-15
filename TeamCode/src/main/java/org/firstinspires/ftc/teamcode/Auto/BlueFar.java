package org.firstinspires.ftc.teamcode.Auto;

import android.os.Environment;

import androidx.core.os.TraceKt;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.OutTakeMTI;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import java.io.File;
import java.io.IOException;

@Autonomous(name = "BlueFar", preselectTeleOp = ".pipers \uD83C\uDF36", group = "auto")
public class BlueFar extends LinearOpMode {
    public static Pose2d
    LeftPurple = new Pose2d(0, 0, 0),
    MiddlePurple = new Pose2d(0, 0, 0),
    RightPurple = new Pose2d(0, 0, 0),

    LeftYellow = new Pose2d(0, 0, 0),
    MiddleYellow = new Pose2d(0, 0, 0),
    RightYellow = new Pose2d(0, 0, 0),

    Stack = new Pose2d(0, 0, 0),
    StackSplinePoint = new Pose2d(0, 0, 0),
    BackDrop = new Pose2d(0, 0, 0),
    BackdropSpinePoint = new Pose2d(0, 0, 0);

    public static SampleMecanumDriveCancelable drive;
    private OutTakeMTI outtake;
    private Intake in;

    private int intake = 0, pixelsInStack = 5;

    @Override
    public void runOpMode() throws InterruptedException {
        File file = new File(Environment.getExternalStorageDirectory(), OutTakeMTI.cacheFileName);

        if(file.exists()){
            file.delete();
        }
        try {
            file.createNewFile();
        } catch (IOException e) {
            RobotLog.e("file not found");
        }

        drive = new SampleMecanumDriveCancelable(hardwareMap);
        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub eh = new ExpansionHub(hardwareMap, drive.getLocalizer());
        Controls cn = new Controls(gamepad1, gamepad2);

        outtake = new OutTakeMTI();
        in = new Intake();

        TrajectorySequence leftPurple = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {
                    outtake.setToPurplePlacing();
                })
                .lineToLinearHeading(LeftPurple)
                .addTemporalMarker(() -> {
                    Controls.DropRight = true;
                    Controls.DropRightAck = false;
                })
                .lineToLinearHeading(Stack)

                .build();

        TrajectorySequence leftYellow = drive.trajectorySequenceBuilder(new Pose2d())
                .splineToSplineHeading(StackSplinePoint, 0)
                .splineToSplineHeading(BackdropSpinePoint, 0)
                .splineToSplineHeading(LeftYellow, Math.toRadians(0))
                .addTemporalMarker(() -> {
                    Controls.DropLeft = true;
                    Controls.DropRight = true;
                    Controls.DropRightAck = false;
                    Controls.DropLeftAck = false;

                })
                .waitSeconds(0.001)
                .build();

        TrajectorySequence goToStack = drive.trajectorySequenceBuilder(new Pose2d())
                .splineToSplineHeading(BackdropSpinePoint, 0)
                .splineToSplineHeading(StackSplinePoint, 0)
                .addTemporalMarker(() -> {
                    intake = 1;
                    in.setPixelStackPosition(pixelsInStack);
                })
                .splineToSplineHeading(Stack, 0)

                .build();

        TrajectorySequence takeFromStack = drive.trajectorySequenceBuilder(new Pose2d())
                .addTemporalMarker(() -> {

                })

                .build();

        while(opModeInInit()){
            outtake.update();
        }

        while (opModeIsActive()){
            ControlHub.ControlHubModule.clearBulkCache();
            ExpansionHub.ExpansionHubModule.clearBulkCache();


            outtake.update();
            in.update();
            drive.update();
            cn.loop();
        }

    }
}
