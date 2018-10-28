package org.firstinspires.ftc.teamcode.Backbone;

import android.graphics.Bitmap;
import android.util.Log;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;

import org.corningrobotics.enderbots.endercv.OpenCVPipeline;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfInt;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

public class SampleOrderDetector extends OpenCVPipeline {

    private Mat hsv = new Mat();
    private Mat thresholded = new Mat();
    private Mat blurMat = new Mat();
    private List<MatOfPoint> contours = new ArrayList<>();
    private List<MatOfPoint> filteredContours = new ArrayList<>();
    public double CubePosX;
    public double CubePosY;
    private boolean showContours = true;
    //TODO: TUNE ALL OF THESE
    double filterContoursMinArea = 50.0;
    double filterContoursMinPerimeter = 25.0;
    double filterContoursMinWidth = 125.0;
    double filterContoursMaxWidth = 1000;
    double filterContoursMinHeight = 125.0;
    double filterContoursMaxHeight = 1000;
    double[] filterContoursSolidity = {0.0, 100};
    double filterContoursMaxVertices = 1000000;
    double filterContoursMinVertices = 0.0;
    double filterContoursMinRatio = 0;
    double filterContoursMaxRatio = 1000;
    //public boolean RectExists;
    public Rect boundingRect;
    Mat kernel = new Mat();
    Point anchor = new Point(-1,-1);
    //Scalar borderValue = new Scalar(-1);



    @Override
    public Mat processFrame(Mat rgba, Mat gray){
        //rgba = readFrame();
        Imgproc.cvtColor(rgba,blurMat,Imgproc.COLOR_RGB2HSV);
        Imgproc.blur(blurMat,hsv,new Size(7.41,7.41));//TODO: TUNE THE SIZE VALUES DEPENDING ON ACCURACY OF THE CODE
        Core.inRange(hsv,new Scalar(15,183,83),new Scalar(53,255,255),thresholded);//TODO: TUNE THE SCALAR RANGES DEPENDING ON ACCURACY OF THE CODE
        Imgproc.dilate(thresholded,thresholded,kernel,anchor,20);
        contours = new ArrayList<>();
        Imgproc.findContours(thresholded,contours,new Mat(),Imgproc.RETR_LIST,Imgproc.CHAIN_APPROX_SIMPLE);
        filterContours(contours,filterContoursMinArea,filterContoursMinPerimeter,filterContoursMinWidth,filterContoursMaxWidth,filterContoursMinHeight,filterContoursMaxHeight,filterContoursSolidity,filterContoursMaxVertices,filterContoursMinVertices,filterContoursMinRatio,filterContoursMaxRatio,filteredContours);
        for(int i = 0; i<filteredContours.size(); i++){
            boundingRect = Imgproc.boundingRect(filteredContours.get(i));
            CubePosX = (boundingRect.x+ boundingRect.width)/2;
            CubePosY =(boundingRect.y + boundingRect.height)/2;
            Imgproc.rectangle(rgba , boundingRect.tl(), boundingRect.br(), new Scalar(0, 255, 0), 2, 8);
        }
        return rgba;
    }
    private void filterContours(List<MatOfPoint> inputContours, double minArea,
                                double minPerimeter, double minWidth, double maxWidth, double minHeight, double
                                        maxHeight, double[] solidity, double maxVertexCount, double minVertexCount, double
                                        minRatio, double maxRatio, List<MatOfPoint> output) {
        final MatOfInt hull = new MatOfInt();
        output.clear();
        //operation
        for (int i = 0; i < inputContours.size(); i++) {
            final MatOfPoint contour = inputContours.get(i);
            final Rect bb = Imgproc.boundingRect(contour);
            if (bb.width < minWidth || bb.width > maxWidth) continue;
            if (bb.height < minHeight || bb.height > maxHeight) continue;
            final double area = Imgproc.contourArea(contour);
            if (area < minArea) continue;
            if (Imgproc.arcLength(new MatOfPoint2f(contour.toArray()), true) < minPerimeter) continue;
            Imgproc.convexHull(contour, hull);
            MatOfPoint mopHull = new MatOfPoint();
            mopHull.create((int) hull.size().height, 1, CvType.CV_32SC2);
            for (int j = 0; j < hull.size().height; j++) {
                int index = (int)hull.get(j, 0)[0];
                double[] point = new double[] { contour.get(index, 0)[0], contour.get(index, 0)[1]};
                mopHull.put(j, 0, point);
            }
            final double solid = 100 * area / Imgproc.contourArea(mopHull);
            if (solid < solidity[0] || solid > solidity[1]) continue;
            if (contour.rows() < minVertexCount || contour.rows() > maxVertexCount)	continue;
            final double ratio = bb.width / (double)bb.height;
            if (ratio < minRatio || ratio > maxRatio) continue;
            output.add(contour);
        }
    }
}