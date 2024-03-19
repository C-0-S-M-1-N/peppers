package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleTalk")
public class TeleTalk extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        boolean t = false;
        while (opModeIsActive()){
            if (!t) {
                telemetry.speak("fraier");
                t = true;
            }
            telemetry.update();
        }
    }
}
