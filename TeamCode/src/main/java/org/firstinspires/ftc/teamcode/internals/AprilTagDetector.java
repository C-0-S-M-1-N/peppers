package org.firstinspires.ftc.teamcode.internals;

    import com.qualcomm.robotcore.hardware.HardwareMap;

        import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
        import org.openftc.apriltag.AprilTagDetection;
        import org.openftc.easyopencv.OpenCvCamera;
        import org.openftc.easyopencv.OpenCvCameraFactory;
        import org.openftc.easyopencv.OpenCvCameraRotation;


        import java.util.ArrayList;

public class AprilTagDetector {
    private static OpenCvCamera camera = null;
    private static AprilTagDetectionPipeline pipeline;
    public static double fx = 822.317;
    public static double fy = 822.317;
    public static double cx = 319.495;
    public static double cy = 242.502;

    public static double tagSize = 0.05;
    public static int ID = 0;
    public static void init(HardwareMap hardwareMap)
    {
        int cameraMonitorViewId = hardwareMap.appContext
                .getResources()
                .getIdentifier("cameraMonitorViewId",
                        "id",
                        hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new AprilTagDetectionPipeline(tagSize, fx, fy, cx, cy);
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //TODO ADJUST CAMERA
                camera.startStreaming(640,480, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode)
            {

            }
        });
    }

    public static void stop() { camera.closeCameraDevice(); }

    public static AprilTagDetection[] getDetections()
    {
        ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();
        AprilTagDetection[] detected = new AprilTagDetection[currentDetections.size()];
        for(int i = 0; i < currentDetections.size(); i++) {
            detected[i] = currentDetections.get(i);
        }
        return detected;
    }
}
