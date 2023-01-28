package org.firstinspires.ftc.teamcode.OpModes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.OpenCV.JunctionPipeline;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

@Autonomous
public class JunctionDetection extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        OpenCvCamera camera;
        String webcamName = "Webcam 1";


        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, webcamName), cameraMonitorViewId);

        JunctionPipeline detector = new JunctionPipeline();

        camera.setPipeline(detector);

        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(320,240, OpenCvCameraRotation.UPRIGHT);
            }

            @Override
            public void onError(int errorCode) {}
        });

        while (!isStarted()) {


            // displays maxX and maxY
            telemetry.addData("X value: ", detector.maxX);
            telemetry.addData("Y value: ", detector.maxY);

            telemetry.update();
            /*
            if (detector.length == 0) {


            } else {

                // need to get bounding boxes here
                for (int i = 0; i < detector.boxX.length; i++) {

                    telemetry.addData("X value: ", detector.boxX[i]);
                    telemetry.addData("Y value: ", detector.boxY[i]);

                    telemetry.update();

                }
            }
            */



        }

        waitForStart();
    }
}