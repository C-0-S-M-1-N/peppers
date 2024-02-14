package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.sun.tools.javac.util.Pair;

import org.apache.commons.math3.geometry.euclidean.twod.Line;
import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.Parts.MecanumDrive;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.firstinspires.ftc.teamcode.utils.HoldPosition;

import java.util.Vector;

@TeleOp(name = "tuneHoldPos")
public class TuneHoldPosition extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException{
        ControlHub ch = new ControlHub(hardwareMap);
        ExpansionHub eh = new ExpansionHub(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        MecanumDrive md = new MecanumDrive(telemetry);
        HoldPosition hp = new HoldPosition(telemetry, hardwareMap);
        hp.holdPos(0, 0, 0);

        waitForStart();
        while (opModeIsActive() && !isStopRequested()){
            double[] arr = hp.update();
            md.update(arr[0], arr[1], arr[2] > 0 ? arr[2] : 0, arr[2] < 0 ? -arr[2] : 0, true);

            telemetry.update();
        }

    }
}
