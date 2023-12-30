package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Components.Elevator;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.MecanumDrive;
import org.firstinspires.ftc.teamcode.Parts.OutTake;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;

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
        ExpansionHub eh = new ExpansionHub(hardwareMap);
        ControlHub ch = new ControlHub(hardwareMap);

        FtcDashboard a = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, a.getTelemetry());

        mecanumDrive = new MecanumDrive(telemetry, hardwareMap);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){


            mecanumDrive.update(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger, gamepad1.left_trigger);
            telemetry.addLine(gamepad1.toString());
            telemetry.update();
        }
    }
}