package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Parts.MecanumDrive;

@TeleOp(name = "pipers \uD83C\uDF36️")
public class MainOp extends LinearOpMode {

    public MecanumDrive masina;
    @Override
    public void runOpMode(){

        masina = new MecanumDrive(hardwareMap);
        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            masina.mergi(gamepad1.left_stick_y, gamepad1.left_trigger - gamepad1.right_trigger, gamepad1.left_stick_x);
            telemetry.update();
        }
    }
}