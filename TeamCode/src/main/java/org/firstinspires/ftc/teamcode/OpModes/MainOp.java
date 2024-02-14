package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Components.Elevator;
import org.firstinspires.ftc.teamcode.Components.Hang;
import org.firstinspires.ftc.teamcode.Parts.Avion;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.MecanumDrive;
import org.firstinspires.ftc.teamcode.Parts.OutTake;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ENCODER_PORTS;
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
    public static Controls c;
    public static Hang hang;
    public static Avion avion;
    ElapsedTime time = new ElapsedTime();

    @Override
    public void runOpMode(){
        ControlHub c = new ControlHub(hardwareMap);
        ExpansionHub e = new ExpansionHub(hardwareMap);
        Controls ctr = new Controls(gamepad1, gamepad2);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        ControlHub.telemetry = telemetry;

        outTake = new OutTake(hardwareMap);
        intake = new Intake();
        mecanumDrive = new MecanumDrive(telemetry);

        waitForStart();
        time.reset();
        while(opModeIsActive() && !isStopRequested()){



//            ControlHub.teleMotorCurrents(telemetry);
//            ExpansionHub.teleMotorCurrents(telemetry);


            mecanumDrive.update(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger, gamepad1.left_trigger,
                    gamepad1.a);

            outTake.update();
            intake.update();

            outTake.update_values();
            intake.update_values();

            outTake.runTelemetry();
            intake.runTelemetry();

            ctr.loop();
            telemetry.addData("Hz", 1/time.seconds());
            time.reset();
            telemetry.update();
        }
    }
}