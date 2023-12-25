package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.utils.AutoGamepad;

@Config
@TeleOp(name = "gamepadTest")
public class GamepadTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        AutoGamepad gamepad = new AutoGamepad(gamepad1);

        waitForStart();
        while(opModeIsActive() && !isStopRequested()){

            telemetry.addLine(gamepad.wasPressed.toString());
            telemetry.addLine(gamepad.wasReleased.toString());

            telemetry.update();
        }
    }
}
