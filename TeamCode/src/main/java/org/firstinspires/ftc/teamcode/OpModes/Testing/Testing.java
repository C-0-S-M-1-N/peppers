package org.firstinspires.ftc.teamcode.OpModes.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Parts.ducks.test;
@Config
@TeleOp(name = "testing")
public class Testing extends LinearOpMode {
private test test;
public static double power1, power2, power3, power4;
@Override
    public void runOpMode(){
    test = new test(hardwareMap, telemetry, "lift1", "lift2", "lift3", "x");
    telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    waitForStart();
    while (opModeIsActive() && !isStopRequested()){
        test.setPawa(power1, power2, power3, power4);
    }
}
}
