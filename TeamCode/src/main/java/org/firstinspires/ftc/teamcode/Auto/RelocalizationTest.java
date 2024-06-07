package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDriveCancelable;
import org.firstinspires.ftc.teamcode.internals.AprilTagDetector;
import org.firstinspires.ftc.teamcode.utils.UltraSonicSensor;

import java.util.concurrent.atomic.AtomicReference;

@TeleOp(name = "Relocalization Test")
@Config
public class RelocalizationTest extends LinearOpMode {
    SampleMecanumDriveCancelable drive;
    public static int DesiredId = 5;
    @Override
    public void runOpMode() throws InterruptedException {
        drive = new SampleMecanumDriveCancelable(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        double readValue = 0;

        Rev2mDistanceSensor sensor = hardwareMap.get(Rev2mDistanceSensor.class, "redSensor");



        waitForStart();


        long time = System.currentTimeMillis();

        while (!isStopRequested()){
            telemetry.addData("distance", sensor.getDistance(DistanceUnit.INCH));
            telemetry.addData("freq", 1000.f / ((System.currentTimeMillis() - time)));
            time = System.currentTimeMillis();

            drive.update();
            telemetry.update();
        }
        AprilTagDetector.camera.closeCameraDevice();
    }
}
