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
    public static double fx = 578.272;
    public static double fy = 578.272;
    public static double cx = 402.145;
    public static double cy = 221.506;

    public static double tagSize = 0.166;
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

    //TODO UPDATE FOR CUSTOM RANGE OF TAGS
    public static int getCase()
    {
        ArrayList<AprilTagDetection> currentDetections = pipeline.getLatestDetections();
        if(currentDetections.size() > 0) {
            for (AprilTagDetection tag : currentDetections) {
                ID = tag.id;
                break;
            }
        }

        return ID;
    }
    public static String showCase()
    {
        int detectedCase = getCase();
        if(detectedCase == 1)
            return "STANGA";
        if(detectedCase == 3)
            return "FATA";
        return "DREAPTA";
    }
    public static void CloseCamera()
    {
        camera.closeCameraDevice();
    }

}
