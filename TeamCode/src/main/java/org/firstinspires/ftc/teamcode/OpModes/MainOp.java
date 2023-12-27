package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Elevator;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.MecanumDrive;
import org.firstinspires.ftc.teamcode.Parts.OutTake;

/*
* MAP
*   CH:
*       - M1 + M2 -> elevator   E1 -> elevator
*       - M3 -> intake
*       - M4 -> hang
*
*       - S0 -> virtual1
*       - S2 -> virtual2
*   EH:
*       - M1 + M2 + M3 + M4 -> chassis
*
*       - S0 + S1 -> grippers
*       - S2 -> pivot
*       - S3 -> pixelRotation
*
* */
@TeleOp(name = "pipers \uD83C\uDF36️")
@Config
public class MainOp extends LinearOpMode {

    public static Intake intake;
    public static OutTake outTake;
    public static MecanumDrive mecanumDrive;

    @Override
    public void runOpMode(){


        FtcDashboard a = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, a.getTelemetry());

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){

            telemetry.update();
        }
    }
}