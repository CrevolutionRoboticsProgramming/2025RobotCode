package frc.robot.vision.commands;

import static frc.robot.vision.VisionConfig.camNames;
import static frc.robot.vision.VisionConfig.robotToCamTransforms;

import java.util.Arrays;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.vision.Vision;
import frc.robot.vision.VisionConfig.*;
import frc.robot.vision.VisionConsumer;

public class PhotonEstimatorCommand extends Command{
    private final Vision[] visions;
    private final VisionConsumer visionConsumer;

    public PhotonEstimatorCommand(VisionConsumer consumer){
        visions = new Vision[camNames.length];
        for (int i = 0; i < camNames.length; i++){
            visions[i] = new Vision(camNames[i], robotToCamTransforms[i]);
        }
        this.visionConsumer = consumer;
    }

    @Override
    public void execute(){
        Arrays.stream(visions).forEach(vision -> {
            var visionEst = vision.getEstimatedGlobalPose();
            
            visionEst.ifPresent(est -> {
                var estStdDevs = vision.getEstimationStdDevs();
                
                visionConsumer.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
            });
        });
    }

    @Override
    public boolean runsWhenDisabled(){
        return true;
    }
}
