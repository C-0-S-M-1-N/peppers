package org.firstinspires.ftc.teamcode.Autonomy;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Autos.ObjectDetectionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera2;

@Autonomous
@Config
public class ObjectCaseDetection{


        OpenCvCamera camera;
        public ObjectDetectionPipeline detector;
        public ObjectCaseDetection(HardwareMap hardwareMap, Telemetry telemetry)
        {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier
                    ("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createInternalCamera2(OpenCvInternalCamera2.CameraDirection.BACK, cameraMonitorViewId);
            detector = new ObjectDetectionPipeline(telemetry);
            camera.setPipeline(detector);
            // ------------------ OpenCv code
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {

                @Override
                public void onOpened() {
                    camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode) {
                    // ------------------ Tzeapa frate
                }

            });
            FtcDashboard.getInstance().startCameraStream(camera, 0);
        }
        public ObjectDetectionPipeline.Location getLocation() {
            switch (detector.getLocation()) {
                case LEFT:
                    return ObjectDetectionPipeline.Location.LEFT;
                case MIDDLE:
                    return ObjectDetectionPipeline.Location.MIDDLE;
                case RIGHT:
                    return ObjectDetectionPipeline.Location.RIGHT;
                default:
                    return null;
            }
        }

        public void stop(){
            camera.stopStreaming();
            detector.release();
        }
}