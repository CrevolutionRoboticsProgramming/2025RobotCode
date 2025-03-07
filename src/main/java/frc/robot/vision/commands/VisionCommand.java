// package frc.robot.vision.commands;

// import static frc.robot.vision.VisionConfig.camNames;
// import static frc.robot.vision.VisionConfig.robotToCamTransforms;

// import java.nio.channels.NetworkChannel;

// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.StructPublisher;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.vision.Vision;
// import frc.robot.vision.VisionConsumer;

// public class VisionCommand extends Command {
//     private final Vision[] visions;
//     private final VisionConsumer visionConsumer;

//     public VisionCommand(VisionConsumer consumer){
//         visions = new Vision[camNames.length];
//         for (int i = 0; i < camNames.length; i++){
//             visions[i] = new Vision(camNames[i], robotToCamTransforms[i]);
//         }
//         this.visionConsumer = consumer; 
//     }

//     @Override
//     public void execute(){
//         for (int i = 0; i < visions.length; i++){
//             var vision = visions[i];
//             var visionEst = vision.getEstimatedGlobalPose();
//             visionEst.ifPresent(est -> {
//                 var estStdDevs = vision.getEstimationStdDevs();
//                 visionConsumer.addVisionMeasurement(est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
//             });
//         }
//     }

//     @Override
//     public boolean runsWhenDisabled(){
//         return true;
//     }
// }
