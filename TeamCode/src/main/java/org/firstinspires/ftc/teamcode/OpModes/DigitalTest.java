package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.utils.BetterColorRangeSensor;

@TeleOp(name = "digitalTest")
public class DigitalTest extends LinearOpMode {
    private void a(int x){
        x = (byte)x;
        while (x > 0){
            telemetry.addData("a", x % 2);
            x /= 2;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException{

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        BetterColorRangeSensor sensor = hardwareMap.get(BetterColorRangeSensor.class, "leftSensor");
        sensor.setThresHold(100);
//        ColorRangeSensor sensor = hardwareMap.get(ColorRangeSensor.class, "rightSensor");

        int cnt = 0;

        waitForStart();
        ElapsedTime timeB = new ElapsedTime(), timeN = new ElapsedTime();
        timeB.reset();
        timeN.reset();
//        int val = sensor.getManufacturerRawID();


        while (opModeIsActive() && !isStopRequested()){
//            telemetry.addData("ID", val);
            telemetry.addData("value", sensor.getProximityDistance());
            telemetry.addData("trigger", sensor.LogicProximityStatus());
            telemetry.addData("main", sensor.getDeviceMainCtrl());
//            telemetry.addData("value", sensor.getDistance(DistanceUnit.MM));
            telemetry.update();


        }
    }
}
