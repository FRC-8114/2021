package frc.robot.subsystems;

import org.opencv.core.Mat;
import org.opencv.videoio.VideoCapture;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SearchSystem extends SubsystemBase {
    static GripPipeline2 pipeline = new GripPipeline2();

    VideoCapture camera = new VideoCapture(0);
    static Mat frame = new Mat();
    public SearchSystem() {
        camera.read(frame);
    }

    public static void process() {
        pipeline.process(frame);
    }
}