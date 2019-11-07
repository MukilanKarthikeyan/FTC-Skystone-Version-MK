package org.firstinspires.ftc.teamcode;

import com.disnodeteam.dogecv.detectors.skystone.SkystoneDetector;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.Locale;
import java.util.Timer;

import static java.lang.String.*;
import static org.opencv.core.CvType.CV_8UC1;

public class Camera {
    private final LinearOpMode opMode;

    //DogeCV init stuff
    OpenCvCamera phoneCam;
    SkystoneDetector skyStoneDetector;

    //EasyOpenCV init
    int left_hue;
    int right_hue;

    int left_br;
    int right_br;

    int pattern;

    OpenCvCamera phoneCamera;
    SamplePipeline stone_pipeline;

    public Camera(LinearOpMode opMode) {
        this.opMode = opMode;

        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        phoneCamera = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCamera.openCameraDevice();

        stone_pipeline = new SamplePipeline();
        phoneCamera.setPipeline(stone_pipeline);

        phoneCamera.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
    }

    public void DogeCV() {
        OpenCvCamera phoneCam;
        SkystoneDetector skyStoneDetector;
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        skyStoneDetector = new SkystoneDetector();
        phoneCam.setPipeline(skyStoneDetector);

        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPSIDE_DOWN);
        opMode.waitForStart();

        while (opMode.opModeIsActive()) {
            /*
             * Send some stats to the telemetry
             */
            opMode.telemetry.addData("Stone Position X", skyStoneDetector.getScreenPosition().x);
            opMode.telemetry.addData("Stone Position Y", skyStoneDetector.getScreenPosition().y);
            opMode.telemetry.addData("Frame Count", phoneCam.getFrameCount());
            opMode.telemetry.addData("FPS", format(Locale.US, "%.2f", phoneCam.getFps()));
            opMode.telemetry.addData("Total frame time ms", phoneCam.getTotalFrameTimeMs());
            opMode.telemetry.addData("Pipeline time ms", phoneCam.getPipelineTimeMs());
            opMode.telemetry.addData("Overhead time ms", phoneCam.getOverheadTimeMs());
            opMode.telemetry.addData("Theoretical max FPS", phoneCam.getCurrentPipelineMaxFps());
            opMode.telemetry.update();
        }
    }

    public double[] findSkyStone() {
        initDogeCV();
        boolean SkyStoneFound = skyStoneDetector.isDetected();
        double[] x_y = {0,0};

        while (!SkyStoneFound) {
            SkyStoneFound = skyStoneDetector.isDetected();
            opMode.telemetry.addData("X-Position: ", skyStoneDetector.getScreenPosition().x);
            opMode.telemetry.addData("Y-Position: ", skyStoneDetector.getScreenPosition().y);
            opMode.telemetry.addData("Skystone found: ", skyStoneDetector.isDetected());
            opMode.telemetry.update();
        }
        x_y[0] = skyStoneDetector.getScreenPosition().x;
        x_y[1] = skyStoneDetector.getScreenPosition().y;

        opMode.telemetry.addData("X-Position: ", x_y[0]);
        opMode.telemetry.addData("Y-Position: ", x_y[1]);
        opMode.telemetry.addData("Skystone found: ", skyStoneDetector.isDetected());
        opMode.telemetry.update();

        return x_y;
    }

    public void initDogeCV() {
        int cameraMonitorViewId = opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", opMode.hardwareMap.appContext.getPackageName());
        phoneCam = new OpenCvInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
        phoneCam.openCameraDevice();
        skyStoneDetector = new SkystoneDetector();
        phoneCam.setPipeline(skyStoneDetector);
        phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
    }

    class SamplePipeline extends OpenCvPipeline {

        @Override
        public Mat processFrame(Mat input) {
            input.convertTo(input, CV_8UC1, 1, 10);

            //telemetry.addData("Input Cols: ", input.cols());
            //telemetry.addData("Input Rows: ", input.rows());
            //telemetry.update();

            int[] left_rect = {
                    (int) (input.cols() * (11f / 32f)),
                    (int) (input.rows() * (12f / 32f)),
                    (int) (input.cols() * (17f / 32f)),
                    (int) (input.rows() * (17f / 32f))
            };

            int[] right_rect = {
                    (int) (input.cols() * (19f / 32f)),
                    (int) (input.rows() * (12f / 32f)),
                    (int) (input.cols() * (25f / 32f)),
                    (int) (input.rows() * (17f / 32f))
            };

            Imgproc.rectangle(
                    input,
                    new org.opencv.core.Point(
                            left_rect[0],
                            left_rect[1]),

                    new org.opencv.core.Point(
                            left_rect[2],
                            left_rect[3]),
                    new Scalar(0, 255, 0), 1);

            Imgproc.rectangle(
                    input,
                    new org.opencv.core.Point(
                            right_rect[0],
                            right_rect[1]),

                    new Point(
                            right_rect[2],
                            right_rect[3]),
                    new Scalar(0, 0, 255), 1);

            Mat left_block = input.submat(left_rect[1], left_rect[3], left_rect[0], left_rect[2]);
            Mat right_block = input.submat(right_rect[1], right_rect[3], right_rect[0], right_rect[2]);


            Scalar left_mean = Core.mean(left_block);


            Scalar right_mean = Core.mean(right_block);

            left_hue = get_hue((int) left_mean.val[0], (int) left_mean.val[1], (int) left_mean.val[2]);
            right_hue = get_hue((int) right_mean.val[0], (int) right_mean.val[1], (int) right_mean.val[2]);
            left_br = get_brightness((int) left_mean.val[0], (int) left_mean.val[1], (int) left_mean.val[2]);
            right_br = get_brightness((int) right_mean.val[0], (int) right_mean.val[1], (int) right_mean.val[2]);

            if (left_br > 70 && right_br > 70) pattern = 1;
            else if (left_br > 70 && right_br < 70) pattern = 3;
            else if (left_br < 70 && right_br > 70) pattern = 2;
            else if (left_br < 70 && right_br < 70) {
                if (left_br > right_br) {
                    pattern = 1;
                } else if (left_br < right_br) {
                    pattern = 2;
                } else {
                    pattern = 3;
                }
            }

            return input;
        }

        private int get_hue(int red, int green, int blue) {

            float min = Math.min(Math.min(red, green), blue);
            float max = Math.max(Math.max(red, green), blue);

            if (min == max) {
                return 0;
            }

            float hue = 0f;
            if (max == red) {
                hue = (green - blue) / (max - min);

            } else if (max == green) {
                hue = 2f + (blue - red) / (max - min);

            } else {
                hue = 4f + (red - green) / (max - min);
            }

            hue = hue * 60;
            if (hue < 0) hue = hue + 360;

            return Math.round(hue);
        }

        private int get_brightness(int red, int green, int blue) {
            return (int) (((double) (red + green + blue)) / 3);
        }

    }

    public int getPattern() {
        return pattern;
    }

}
