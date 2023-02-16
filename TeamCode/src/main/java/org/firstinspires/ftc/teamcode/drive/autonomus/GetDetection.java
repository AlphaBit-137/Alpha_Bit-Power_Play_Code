package org.firstinspires.ftc.teamcode.drive.autonomus;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

public class GetDetection {

    public OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    public int caz = 2;

    double fx = 202;
    double fy = 226;
    double cx = 137;
    double cy = 183;

    int LEFT = 11;
    int MIDDLE = 2;
    int RIGHT = 5;

    AprilTagDetection tagOfInterest = null;

    // UNITS ARE METERS
    double tagsize = 0.036;

    public void initCamera(HardwareMap hardwareMap) {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);

        camera.setPipeline(aprilTagDetectionPipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.SIDEWAYS_LEFT);
            }

            @Override
            public void onError(int errorCode) {
            }
        });

    }

    public void Detect() {
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();

        if (currentDetections.size() != 0) {
            boolean tagFound = false;

            for (AprilTagDetection tag : currentDetections) {
                if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

        }
    }

    public int getCase()
    {
        if(tagOfInterest == null)
        {

        }else if (tagOfInterest.id == LEFT) {
            caz = 1;
        } else if (tagOfInterest.id == RIGHT) {
            caz = 3;
        } else caz = 2;

        return caz;
    }
}
