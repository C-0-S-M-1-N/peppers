package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Components.Elevator;
import org.firstinspires.ftc.teamcode.internals.ControlHub;

@TeleOp(name = "elevator tune")
@Config
public class ElevatorTuning extends LinearOpMode {
    public static Elevator elevator;

    boolean dpad_up, dpad_down;
    public static int pos = 0;

    @Override
    public void runOpMode(){

        ControlHub ch = new ControlHub(hardwareMap);

        elevator = new Elevator(telemetry);


        FtcDashboard a = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, a.getTelemetry());

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){

            if(gamepad1.dpad_up && !dpad_up){
                pos ++ ;
                if(pos > 11) pos = 11;
            }
            if(gamepad1.dpad_down && !dpad_down){
                pos--;
                if(pos < 0) pos = 0;

            }
            dpad_up = gamepad1.dpad_up;
            dpad_down = gamepad1.dpad_down;

            elevator.setPosition(950/11 * pos);
            elevator.update();
            elevator.runTelemetry();

            telemetry.addData("position", pos * 950/11);

            telemetry.update();
        }
    }
}
