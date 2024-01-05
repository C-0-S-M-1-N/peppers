package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Components.Elevator;
import org.firstinspires.ftc.teamcode.Components.ElevatorArm;
import org.firstinspires.ftc.teamcode.Components.PixelBed;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;

@TeleOp(name = "elevator tune")
@Config
public class ElevatorTuning extends LinearOpMode {
    public static Elevator elevator;
    public static ElevatorArm arm;
    public static PixelBed bed;

    boolean dpad_up, dpad_down;
    public static int pos = 0;

    @Override
    public void runOpMode(){

        ControlHub ch = new ControlHub(hardwareMap);
        ExpansionHub eh = new ExpansionHub(hardwareMap);
        FtcDashboard a = FtcDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, a.getTelemetry());

        elevator = new Elevator(telemetry);
        arm = new ElevatorArm(telemetry);
        bed = new PixelBed(telemetry);


        arm.setAngle(0);
        bed.setBedAngle(0);
        bed.setHorizontalRotation();

        arm.update();
        bed.update();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()){

            if(gamepad1.dpad_up && !dpad_up){
                pos ++ ;
                if(pos > 11) pos = 11;
                elevator.setPosition(950/11 * pos);
            }
            if(gamepad1.dpad_down && !dpad_down){
                pos--;
                if(pos < 0) pos = 0;
                elevator.setPosition(950/11 * pos);

            }
            dpad_up = gamepad1.dpad_up;
            dpad_down = gamepad1.dpad_down;

            elevator.update();
            elevator.runTelemetry();

            telemetry.addData("position", pos * 950/11);

            telemetry.update();
        }
    }
}
