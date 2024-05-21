package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.apache.commons.math3.analysis.function.Exp;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.OutTakeMTI;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.utils.AutoServo;

import java.util.ArrayList;

@TeleOp(name = ".pipers \uD83C\uDF36")
@Config
public class TeleTalk extends LinearOpMode {
    SampleMecanumDriveCancelable mecanumDrive;
    @Override
    public void runOpMode() throws InterruptedException {
        mecanumDrive = new SampleMecanumDriveCancelable(hardwareMap);
        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub e = new ExpansionHub(hardwareMap, mecanumDrive.getLocalizer());
        Controls cn = new Controls(gamepad1, gamepad2);
        ControlHub.telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        OutTakeMTI out = new OutTakeMTI();
        Intake intake = new Intake();
        out.update();

        boolean boost = true;

        waitForStart();

        while (opModeIsActive()){
            ControlHub.ControlHubModule.clearBulkCache();
            ExpansionHub.ExpansionHubModule.clearBulkCache();
            if(Controls.ResetTourret) ExpansionHub.resetIMU();

            e.update(false);

            boost = !gamepad1.a;

            mecanumDrive.setWeightedDrivePower(  new Pose2d(
                -gamepad1.left_stick_y * (boost ? 1.0 : 0.3),
                -gamepad1.left_stick_x * (boost ? 1.0 : 0.3),
           (gamepad1.left_trigger - gamepad1.right_trigger) * (boost ? 1.0 : 0.3)
            ));

            out.update();
            intake.update();
            intake.update_values();
            mecanumDrive.update();
            e.teleAngle(ControlHub.telemetry);

            cn.loop();
            ControlHub.telemetry.update();
        }

    }
}
