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
    public boolean isBlue = true;

    public static int leftRectTopX = 0 ,leftRectTopY = 50;
    public static int leftRectBottomX = 80 ,leftRectBottomY = 150;

    public static int middleRectTopX = 220 ,middleRectTopY = 70;
    public static int middleRectBottomX = 310 ,middleRectBottomY = 155;

    public static int lowH = 100 ,lowS = 80, lowV = 50;
    public static int highH = 140, highS = 255, highV = 255;

    public static double tseThreshold = 0.18;

    public ObjectDetectionPipeline(Telemetry telemetry, boolean b) {this.telemetry = telemetry; isBlue = b;}

    @Override
    public Mat processFrame(Mat input){

        Rect LEFT_ROI = new Rect(
                new Point(leftRectTopX, leftRectTopY),
                new Point(leftRectBottomX, leftRectBottomY));
        Rect MIDDLE_ROI = new Rect(
                new Point(middleRectTopX, middleRectTopY),
                new Point(middleRectBottomX, middleRectBottomY));

        if(isBlue)
            Imgproc.cvtColor(input, mat, Imgproc.COLOR_RGB2HSV);
        else
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
        telemetry.addData("Middle percentage", Math.round(middleValue * 100) + "%");

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