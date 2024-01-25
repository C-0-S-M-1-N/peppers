package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.internals.AprilTagDetector;

@TeleOp(name = "aprilTagDetection")
@Config
public class AprilTag extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        AprilTagDetector.init(hardwareMap);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            telemetry.addData("case", AprilTagDetector.getCase());
            telemetry.update();
        }
    }
}
