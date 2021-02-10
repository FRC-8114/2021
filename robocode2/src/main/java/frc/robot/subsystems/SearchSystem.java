package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SearchSystem extends SubsystemBase {
    static GripPipeline pipeline = new GripPipeline();
    static CvSink cvSink;
    // VideoCapture camera = new VideoCapture(0);
    static Mat frame = new Mat();

    public SearchSystem() {
        CameraServer.getInstance().startAutomaticCapture();
        cvSink = CameraServer.getInstance().getVideo();
        CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 640, 480);

    }

    public static void process() {

        pipeline.process(frame);
    }
}