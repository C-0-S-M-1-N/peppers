package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Components.ElevatorArm;
import org.firstinspires.ftc.teamcode.Components.PixelBed;
import org.firstinspires.ftc.teamcode.Parts.OutTake;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;

@Config
@TeleOp(name = "Tune")
public class Tune extends LinearOpMode {
    public static OutTake outTake;
    public static Controls controls;
    public static double angle = 0, pos = 0;
    public static ElapsedTime time;
    @Override
    public void runOpMode() throws InterruptedException{

        telemetry = new MultipleTelemetry(FtcDashboard.getInstance().getTelemetry(), telemetry);

        ExpansionHub eh = new ExpansionHub(hardwareMap);
        ControlHub ch = new ControlHub(hardwareMap);
        time = new ElapsedTime();
        controls = new Controls(gamepad1, gamepad2);

        outTake = new OutTake(hardwareMap, telemetry);

        waitForStart();
        time.reset();
        while(!isStopRequested() && opModeIsActive()){
            outTake.update();
            outTake.update_values();

            outTake.runTelemetry();
            telemetry.addData("freq", 1/time.seconds());
            time.reset();
            telemetry.update();
            controls.loop();
        }
    }
}
