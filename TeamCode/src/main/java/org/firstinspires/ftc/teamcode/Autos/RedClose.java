package org.firstinspires.ftc.teamcode.Autos;

import android.annotation.SuppressLint;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.Autos.RedCloseTrajectory;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Components.Controls;
import org.firstinspires.ftc.teamcode.Parts.Intake;
import org.firstinspires.ftc.teamcode.Parts.OutTake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.internals.ControlHub;
import org.firstinspires.ftc.teamcode.internals.ExpansionHub;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.net.IDN;

@Autonomous(group = "Autos", preselectTeleOp = "pipers \\uD83C\\uDF36", name = "AutoRedClose")
@Config
public class RedClose extends LinearOpMode {

    public enum STATES{
        PARK(),
        RETRACT_ELEVATOR(PARK),
        DROP_PIXELS(RETRACT_ELEVATOR),
        RAISE_ELEVATOR(DROP_PIXELS),
        TO_BACKDROP(RAISE_ELEVATOR),
        TO_STACK(TO_BACKDROP),
        TRANSIT_POSITION(TO_BACKDROP),
        PLACE_LEFT(TRANSIT_POSITION),
        PLACE_RIGHT(TRANSIT_POSITION),
        IDLE,
        PLACE_MIDDLE(TRANSIT_POSITION);

        STATES next;

        STATES(STATES next){
            this.next = next;
        }
        STATES(){
            this.next = null;
        }
    }

    STATES State = STATES.PLACE_RIGHT;

    private org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive mecanumDrive;
    RedCloseTrajectory t;
    OpenCvCamera camera;

    int q;

    private boolean updateMecanum = true, reachedBackdrop = false, retreatElevator = false, goingPreload = true;
    private boolean action_done = true;


    @SuppressLint("SuspiciousIndentation")
    public void runOpMode() throws InterruptedException{
        ControlHub ch = new ControlHub(hardwareMap);
        ExpansionHub eh = new ExpansionHub(hardwareMap);
        mecanumDrive = new SampleMecanumDrive(hardwareMap);
        t = new RedCloseTrajectory(mecanumDrive);

        OutTake outTake = new OutTake(hardwareMap, telemetry);
        Intake intake = new Intake();
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        OutTake.useControls = false;

        // ------------------ OpenCv initialisation code
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier
                ("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        ObjectDetectionPipeline detector = new ObjectDetectionPipeline(telemetry, false);

        camera.setPipeline(detector);
        // ------------------ OpenCv code
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

            @Override
            public void onOpened() {
                camera.startStreaming(432, 240, OpenCvCameraRotation.SENSOR_NATIVE);
            }

            @Override
            public void onError(int errorCode) {
                // ------------------ Tzeapa frate
            }

        });
        FtcDashboard.getInstance().startCameraStream(camera, 0);

        waitForStart();

        switch (detector.getLocation()) {
            case LEFT:
                telemetry.addLine("LEFT CASE");
                State = STATES.PLACE_LEFT;
                q = 1;
                break;
            case MIDDLE:
                telemetry.addLine("MIDDLE CASE");
                State = STATES.PLACE_MIDDLE;
                q = 2;
                break;
            case RIGHT:
                telemetry.addLine("RIGHT CASE");
                State = STATES.PLACE_RIGHT;
                q = 3;
                break;
            default:
                // not used
                telemetry.addData("location" , "nu stiu boss");
                mecanumDrive.followTrajectorySequenceAsync(t.PreloadL());
                break;
        }
        camera.stopStreaming();

        while(opModeIsActive() && !isStopRequested()){
            if(State == null) State = STATES.IDLE;
            if(!mecanumDrive.isBusy() && outTake.STATE == OutTake.STATES.IDLE) action_done = true;
                switch (State) {
                    case PLACE_LEFT:
                        mecanumDrive.followTrajectorySequence(t.PreloadL());
                        break;
                    case PLACE_RIGHT:
                        mecanumDrive.followTrajectorySequence(t.PreloadR());
                        break;
                    case PLACE_MIDDLE:
                        mecanumDrive.followTrajectorySequence(t.PreloadM());
                        break;
                    case TRANSIT_POSITION:
                        mecanumDrive.followTrajectorySequence(t.Transit());
                        break;
                    case TO_BACKDROP:
                        switch (q){
                            case 1:mecanumDrive.followTrajectorySequence(t.LBackdrop()); telemetry.addData("q",q); break;
                            case 2:mecanumDrive.followTrajectorySequence(t.MBackdrop()); telemetry.addData("q",q); break;
                            case 3:mecanumDrive.followTrajectorySequence(t.RBackdrop()); telemetry.addData("q",q); break;
                        }
                        outTake.STATE = OutTake.STATES.EXTEND_TRIGGER;
                        OutTake.STATES.currentLevel = 2; // TODO: TUNE!!!
                        outTake.update();
                        break;
                    case RAISE_ELEVATOR:
                        break;
                    case DROP_PIXELS:
                        Controls.DropRight = true;
                        Controls.DropLeft = true;
                        break;
                    case RETRACT_ELEVATOR:
                        outTake.STATE = OutTake.STATES.RETRACT_TRIGGER;
                        outTake.update();
                        break;
                    case PARK:
                        mecanumDrive.followTrajectorySequence(t.Park());
                        outTake.STATE = OutTake.STATES.RETRACT_TRIGGER;
                        break;
                    case IDLE:
                        break;
                    default:
                        break;
                }
                if(State != STATES.PARK && State != null)
                    State = State.next;
                else State = STATES.IDLE;


            mecanumDrive.update();
            intake.update();
            outTake.update();
            outTake.runTelemetry();
//            telemetry.addData("STATE", State.toString());
//            telemetry.update();
        }
}}