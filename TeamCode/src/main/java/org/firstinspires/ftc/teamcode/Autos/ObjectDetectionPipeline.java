package org.firstinspires.ftc.teamcode.Autos;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

@Config
public class ObjectDetectionPipeline extends OpenCvPipeline {
    Mat mat = new Mat();
    Telemetry telemetry;
    public enum Location{
        LEFT,
        MIDDLE,
        RIGHT
    }
    private Location location = Location.LEFT;

    public static int leftRectTopX = 10 ,leftRectTopY = 120;
    public static int leftRectBottomX = 50 ,leftRectBottomY = 240;

    public static int middleRectTopX = 150 ,middleRectTopY = 150;
    public static int middleRectBottomX = 200 ,middleRectBottomY = 200;

    public static int lowH = 100 ,lowS = 80, lowV = 50;
    public static int highH = 140, highS = 255, highV = 255;

    public static double tseThreshold = 0.1;

    static final Rect LEFT_ROI = new Rect(
            new Point(leftRectTopX, leftRectTopY),
            new Point(leftRectBottomX, leftRectBottomY));
    static final Rect MIDDLE_ROI = new Rect(
            new Point(middleRectTopX, middleRectTopY),
            new Point(middleRectBottomX, middleRectBottomY));

    public ObjectDetectionPipeline(Telemetry telemetry, boolean b) {this.telemetry = telemetry;}

    @Override
    public Mat processFrame(Mat input) {
        //thresholding
        Imgproc.cvtColor(input, mat, Imgproc.COLOR_BGR2HSV);
        Scalar lowHSV = new Scalar(lowH ,lowS, lowV);
        Scalar highHSV = new Scalar(highH ,highS, highV);
        Core.inRange(mat, lowHSV, highHSV ,mat);

        Mat left = mat.submat(LEFT_ROI);
        Mat middle = mat.submat(MIDDLE_ROI);

        double leftValue = Core.sumElems(left).val[0] /LEFT_ROI.area() /255;
        double middleValue = Core.sumElems(middle).val[0] /MIDDLE_ROI.area() /255;

        left.release();
        middle.release();

        boolean tseLeft = leftValue > tseThreshold;
        boolean tseMiddle = middleValue > tseThreshold;

        telemetry.addData("Left raw value", (int) Core.sumElems(left).val[0]);
        telemetry.addData("Right raw value", (int) Core.sumElems(middle).val[0]);
        telemetry.addData("Left percentage", Math.round(leftValue * 100) + "%");
        telemetry.addData("Right percentage", Math.round(middleValue * 100) + "%");

        if(tseLeft) {
            location = Location.LEFT;
            telemetry.addData("pixel_location: ", "left");
        }
        else if(tseMiddle){
            location = Location.MIDDLE;
            telemetry.addData("pixel_location: ", "middle");
        }
        else{
            location = Location.RIGHT;
            telemetry.addData("pixel_location: ", "right");
        }
        telemetry.update();
        Imgproc.cvtColor(mat,mat,Imgproc.COLOR_GRAY2RGB);

        Scalar colorFound = new Scalar(255,0,0);
        Scalar colorNotFound = new Scalar(0,255,0);

        Imgproc.rectangle(mat,LEFT_ROI,location == Location.LEFT? colorFound:colorNotFound);
        Imgproc.rectangle(mat,MIDDLE_ROI,location == Location.MIDDLE? colorFound:colorNotFound);

        return mat;

    }
    public Location getLocation(){
        return location;
    }
    public void release(){
        mat.release();
    }
}