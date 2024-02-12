package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name = "digitalTest")
public class DigitalTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        DigitalChannel s0 = hardwareMap.get(DigitalChannel.class, "cD0");
        DigitalChannel s1 = hardwareMap.get(DigitalChannel.class, "cD1");

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        s0.setMode(DigitalChannel.Mode.INPUT);
        s1.setMode(DigitalChannel.Mode.INPUT);
        waitForStart();

        while (opModeIsActive() && !isStopRequested()){
            telemetry.addData("state0", s0.getState());
            telemetry.addData("state1", s1.getState());
            telemetry.update();
        }
    }
}
