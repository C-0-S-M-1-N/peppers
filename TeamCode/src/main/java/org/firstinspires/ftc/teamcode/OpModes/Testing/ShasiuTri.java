package org.firstinspires.ftc.teamcode.OpModes.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Parts.MDTrig;
@Config
@TeleOp(name = "TriSash")
public class ShasiuTri extends LinearOpMode {
    private MDTrig masina;

    @Override
    public void runOpMode(){
        masina = new MDTrig(hardwareMap, telemetry);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        waitForStart();

        while(opModeIsActive() && !isStopRequested()){
            masina.mergi(-gamepad1.left_stick_y,  gamepad1.right_trigger - gamepad1.left_trigger, gamepad1.left_stick_x, gamepad1.circle);
            telemetry.update();
        }
    }
}
