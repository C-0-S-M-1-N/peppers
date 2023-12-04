package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Elevator;

@TeleOp(name = "pipers \uD83C\uDF36️")
public class MainOp extends LinearOpMode {
    Elevator elevator;
    @Override
    public void runOpMode(){
        elevator = new Elevator(hardwareMap, telemetry);

        FtcDashboard a = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, a.getTelemetry());

        waitForStart();
        boolean prevA = false;

        while(opModeIsActive() && !isStopRequested()){
            if(!prevA && gamepad1.a) {
                elevator.setPosition(800);
            }

            elevator.update();

            prevA = gamepad1.a;
            telemetry.update();
        }
    }
}