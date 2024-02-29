package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.utils.BetterColorRangeSensor;

@TeleOp(name = "digitalTest")
public class DigitalTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{

        BetterColorRangeSensor sensor = hardwareMap.get(BetterColorRangeSensor.class, "leftSensor");

        waitForStart();
        int val = sensor.getManufacturerRawID();
        while (opModeIsActive() && !isStopRequested()){
            telemetry.addData("ID", val);
            if(val == 0xC2){
                telemetry.addLine("MERGEEE!!$#@RGRGE");
            }
            telemetry.update();
        }
    }
}
