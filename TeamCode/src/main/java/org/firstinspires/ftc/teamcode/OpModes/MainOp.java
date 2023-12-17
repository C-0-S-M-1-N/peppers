package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Parts.MecanumDrive;

@TeleOp(name = "pipers \uD83C\uDF36️")
public class MainOp extends LinearOpMode {

    @Override
    public void runOpMode(){
        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            telemetry.update();
        }
    }
}