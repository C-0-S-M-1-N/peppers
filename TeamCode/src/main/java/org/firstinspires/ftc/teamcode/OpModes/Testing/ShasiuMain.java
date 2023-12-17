package org.firstinspires.ftc.teamcode.OpModes.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Parts.MecanumDrive;
@Config
@TeleOp(name = "Robo go brr ️")
public class ShasiuMain extends LinearOpMode {

    public MecanumDrive masina;
    @Override
    public void runOpMode(){
        gamepad1 = new Gamepad();
        masina = new MecanumDrive(hardwareMap, telemetry);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            masina.mergi(-gamepad1.left_stick_y,  gamepad1.right_trigger - gamepad1.left_trigger, gamepad1.left_stick_x);
            telemetry.update();
        }
    }
}