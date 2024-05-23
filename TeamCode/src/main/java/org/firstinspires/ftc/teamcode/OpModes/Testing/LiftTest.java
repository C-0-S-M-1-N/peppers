package org.firstinspires.ftc.teamcode.OpModes.Testing;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Parts.Lift;
@Config
@TeleOp(name = "Lift Test")
public class LiftTest extends LinearOpMode {
    private Lift lift;
    public static int trg;
    public static double ki = 0.0015, kd = 0.0007, kp = 0.025;
    @Override
    public void runOpMode(){
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        lift = new Lift(hardwareMap, telemetry, kp, ki, kd);
        waitForStart();
        lift.initiation();
        while(opModeIsActive() && !isStopRequested()){
            lift.setPidCoefficients(kp, ki, kd);
           // lift.prototip(trg, gamepad1.circle);
            lift.update(gamepad1.dpad_up, gamepad1.dpad_down);
            telemetry.update();
        }
    }
}
