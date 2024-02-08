package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;

@Config
@TeleOp(name = "testHardware")
public class TestHardware extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException{
        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub expansionHub = new ExpansionHub(hardwareMap);
        telemetry = FtcDashboard.getInstance().getTelemetry();

        DigitalChannel d1 = hardwareMap.get(DigitalChannel.class, "eD1");
        DigitalChannel d2 = hardwareMap.get(DigitalChannel.class, "eD0");

        d1.setMode(DigitalChannel.Mode.INPUT);
        d2.setMode(DigitalChannel.Mode.INPUT);

        waitForStart();

        while(!isStopRequested() && opModeIsActive()){
            telemetry.addData("sensor 1 state", d1.getState());
            telemetry.addData("sensor 2 state", d2.getState());

            telemetry.update();
        }

    }
}
