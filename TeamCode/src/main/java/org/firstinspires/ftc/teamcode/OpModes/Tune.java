package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Components.ElevatorArm;
import org.firstinspires.ftc.teamcode.Components.PixelBed;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.internals.MOTOR_PORTS;

@Config
@TeleOp(name = "Tune")
public class Tune extends LinearOpMode {
    public static PixelBed pixelBed;
    public static double angle = 0;
    @Override
    public void runOpMode() throws InterruptedException{
        ExpansionHub eh = new ExpansionHub(hardwareMap);
        ControlHub ch = new ControlHub(hardwareMap);

        pixelBed = new PixelBed(telemetry);

        waitForStart();
        while(!isStopRequested() && opModeIsActive()){
            pixelBed.setBedAngle(angle);
            pixelBed.update();
        }
    }
}
