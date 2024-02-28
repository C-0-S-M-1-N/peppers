package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.apache.commons.math3.analysis.function.Exp;
import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Components.Elevator;
import org.firstinspires.ftc.teamcode.Components.Hang;
import org.firstinspires.ftc.teamcode.Components.OutTakeExtension;
import org.firstinspires.ftc.teamcode.Parts.Avion;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.MecanumDrive;
import org.firstinspires.ftc.teamcode.Parts.OutTake;
import org.firstinspires.ftc.teamcode.drive.StandardTrackingWheelLocalizer;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;

import java.util.ArrayList;


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
        ExpansionHub e = new ExpansionHub(hardwareMap, new StandardTrackingWheelLocalizer(hardwareMap, new ArrayList<>(), new ArrayList<>()));
        Controls ctr = new Controls(gamepad1, gamepad2);

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);
        ControlHub.telemetry = telemetry;
        gamepad1.setLedColor(1.0, 1.0, 1.0, (int) 1e10);

        outTake = new OutTake(hardwareMap);
        intake = new Intake();
        mecanumDrive = new MecanumDrive(telemetry);
        hang = new Hang();
        avion = new Avion();

        OutTakeExtension.MOTION_PROFILED = true;

        waitForStart();
        ExpansionHub.runI2Cdevices.start();
        time.reset();
        while(opModeIsActive() && !isStopRequested()){
            for(LynxModule m : ControlHub.all){
                m.clearBulkCache();
            }


            for(int i = 0; i < 4; i++){
                ControlHub.encoder[i].read = false;
                ExpansionHub.encoder[i].read = false;
            }



//            ControlHub.teleMotorCurrents(telemetry);
//            ExpansionHub.teleMotorCurrents(telemetry);


            mecanumDrive.update(-gamepad1.left_stick_y, gamepad1.left_stick_x, gamepad1.right_trigger, gamepad1.left_trigger,
                    gamepad1.a);

            outTake.update();
            intake.update();
            hang.update();
            avion.update();
            e.update(true);

            outTake.update_values();
            intake.update_values();

            outTake.runTelemetry();
            intake.runTelemetry();

            ctr.loop();
            telemetry.addData("Hz", 1/time.seconds());
            telemetry.addData("STATE", OutTake.state);
            time.reset();

            telemetry.update();
        }
        e.I2CMutex.kill();
    }
}
