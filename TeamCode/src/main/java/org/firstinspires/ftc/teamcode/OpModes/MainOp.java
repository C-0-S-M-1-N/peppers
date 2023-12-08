package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Parts.MecanumDrive;

@TeleOp(name = "pipers \uD83C\uDF36️")
public class MainOp extends LinearOpMode {

    public MecanumDrive masina;
    private Gamepad curGP, prevGP;
    @Override
    public void runOpMode(){
        curGP = new Gamepad();
        prevGP = new Gamepad();
        gamepad1 = new Gamepad();
        masina = new MecanumDrive(hardwareMap);
        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            prevGP.copy(curGP);
            curGP.copy(gamepad1);
            masina.putere();
            if(curGP.a)
            telemetry.addData("schimbat speed", masina.speedDown);
            masina.mergi(-gamepad1.left_stick_y,  gamepad1.right_trigger - gamepad1.left_trigger, gamepad1.left_stick_x);
            telemetry.update();
        }
    }
}