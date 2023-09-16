package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;
import org.openftc.easyopencv.OpenCvWebcam;

import java.util.ArrayList;
import java.util.List;

@Config
public class WebcamSubsystem implements Subsystem {

    public static boolean WEBCAM_STREAMING_ENABLED = true;
    public String webcamName = "wcam";

    public int WEBCAM_WIDTH = 320;
    public int WEBCAM_HEIGHT = 240;

    public static int THRESH_MIN = 100;
    public static int THRESH_MAX = 250;

    public static int TARGET_X = 160;
    public static int TARGET_Y = 100;

    OpenCvWebcam webcam;

    protected int detectedValue = 0;

    protected WebcamSubsystem.LocalPipeline pipeline;

    public WebcamSubsystem(HardwareMap hardwareMap) {
        if (WEBCAM_STREAMING_ENABLED) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        } else {
            webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName));
        }
        pipeline = new WebcamSubsystem.LocalPipeline();
        webcam.setPipeline(pipeline);
        webcam.setMillisecondsPermissionTimeout(5000);
        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming(WEBCAM_WIDTH, WEBCAM_HEIGHT, OpenCvCameraRotation.UPRIGHT);
                FtcDashboard.getInstance().startCameraStream(webcam, 0);
            }

            @Override
            public void onError(int errorCode)
            {
                /*
                 * This will be called if the camera could not be opened
                 */
            }
        });
    }

    @Override
    public void periodic() {
        this.detectedValue = pipeline.getDetectedValue();
    }

    public int getDetectedValue() {
        return detectedValue;
    }

    public int getHorizontalOffset() {
        return pipeline.getHorizontalOffset();
    }

    public int getVerticalOffset() {
        return pipeline.getVerticalOffset();
    }

    class LocalPipeline extends OpenCvPipeline {

        boolean viewportPaused;
        int detectedValue = 0;

        Mat imgGray = new Mat();
        Mat imgThresh = new Mat();

        Mat hierarchy = new Mat();

        Scalar greenColor = new Scalar(0, 255, 0), redColor = new Scalar(255, 0, 0);

        Rect rectOfInterest = new Rect(0,0,0,0);
        private int horizontalOffset = 0;
        private int verticalOffset = 0;

        @Override
        public Mat processFrame(Mat input) {

            List<MatOfPoint> contours = new ArrayList<>();

            Imgproc.cvtColor(input, imgGray, Imgproc.COLOR_BGR2GRAY);
            Imgproc.threshold(imgGray, imgThresh, THRESH_MIN, THRESH_MAX, Imgproc.THRESH_BINARY);
            Imgproc.findContours(imgThresh, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

            MatOfPoint2f[] contoursPoly = new MatOfPoint2f[contours.size()];
            Rect[] boundRect = new Rect[contours.size()];
            for (int i = 0; i < contours.size(); i++) {
                contoursPoly[i] = new MatOfPoint2f();
                Imgproc.approxPolyDP(new MatOfPoint2f(contours.get(i).toArray()), contoursPoly[i], 3, true);
                boundRect[i] = Imgproc.boundingRect(new MatOfPoint(contoursPoly[i].toArray()));
            }

            for (int i = 0; i < contours.size(); i++) {
                if (boundRect[i].width > 25 && boundRect[i].width < (input.cols() / 2)) {
                    rectOfInterest = boundRect[i];
                }
            }

            Imgproc.rectangle(input, rectOfInterest.tl(), rectOfInterest.br(), redColor, 2);
            Imgproc.line(input, new Point(TARGET_X, 0), new Point(TARGET_X, input.rows()), greenColor, 2);
            Imgproc.line(input, new Point(0, TARGET_Y), new Point(input.cols(), TARGET_Y), greenColor, 2);

            horizontalOffset = (int) (rectOfInterest.x + (rectOfInterest.width / 2) - TARGET_X);
            verticalOffset = (int) (rectOfInterest.y + (rectOfInterest.height / 2) - TARGET_Y);

            detectedValue = 1;

            return input;
        }

        @Override
        public void onViewportTapped()
        {
            viewportPaused = !viewportPaused;

            if(viewportPaused)
            {
                webcam.pauseViewport();
            }
            else
            {
                webcam.resumeViewport();
            }
        }

        public int getDetectedValue() {
            return detectedValue;
        }

        public int getHorizontalOffset() {
            return horizontalOffset;
        }

        public int getVerticalOffset() {
            return verticalOffset;
        }
    }

}
