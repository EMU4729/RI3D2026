package frc.robot.subsystems;


import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.opencv.core.Mat.Tuple2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Subsystems;

public class TargetingSub extends SubsystemBase{
  private final Transform3d turretTranslation = new Transform3d(new Translation3d(0.3,0.3,0.4), new Rotation3d(0,0,0));

  private final List<DistanceSample> samples = Arrays.asList(
      new DistanceSample(0.4, Degrees.of(85), Meters.of(0)),
      new DistanceSample(1, Degrees.of(45), Meters.of(15))
  );

  public TargetingSub(){

  }

  public TurretCommand calcTurret(){
    Transform3d transformToTarget = getRRTransform();
    Distance dist = Meters.of(transformToTarget.getTranslation().toTranslation2d().getNorm());
    DistanceSample rangingCommand = calcDistance(dist);

    /*System.out.println(transformToTarget.getTranslation() +" "+
                      transformToTarget.getRotation().toRotation2d().getDegrees() +" "+
                      dist  +" "+
                      rangingCommand.power +" "+
                      rangingCommand.hoodAngle.in(Degrees)
                      );*/

    Subsystems.nav.drawFieldObject("Turret", 
        new Pose2d(
            turretTranslation.getTranslation().toTranslation2d(), 
            transformToTarget.getRotation().toRotation2d()), 
        true);

    return new TurretCommand(Radians.of(transformToTarget.getRotation().getZ()), 
                            rangingCommand.power, 
                            rangingCommand.hoodAngle);
  }

  public DistanceSample calcDistance(Distance dist){
    DistanceSample nearestAbove = samples.get(0);
    DistanceSample nearestBelow = samples.get(0);
    
    double aboveError = Double.MAX_VALUE;
    double belowError = Double.MAX_VALUE;

    for (DistanceSample sample : samples){
      double thisError = (sample.distance.minus(dist)).in(Meters);
      if (thisError >= 0){
        if (Math.abs(thisError) < aboveError){
          nearestAbove = sample;
          aboveError = Math.abs(thisError);
        }
      } else {
        if (Math.abs(thisError) < belowError){
          nearestBelow = sample;
          belowError = Math.abs(thisError);
        }
      }
    }
    if(aboveError == Double.MAX_VALUE || belowError == Double.MAX_VALUE){
      DataLogManager.log("TargetingSub : target out of range");
      return new DistanceSample(0.0, Degrees.of(0), dist);
    }

    double t = (dist.in(Meters) - nearestBelow.distance.in(Meters)) / 
        (nearestAbove.distance.in(Meters) - nearestBelow.distance.in(Meters));

    double power = MathUtil.interpolate(nearestBelow.power, nearestAbove.power, t);
    Angle angle = Radians.of(MathUtil.interpolate(nearestBelow.hoodAngle.in(Radians), 
                                                  nearestAbove.hoodAngle.in(Radians), t));

    return new DistanceSample(power, angle, dist);
  }

  public Transform3d getRRTransform(){
    Pose2d robotPose2d = Subsystems.nav.getPose();
    Pose3d robotPose = new Pose3d(
        new Translation3d(robotPose2d.getX(), robotPose2d.getY(), 0),
        new Rotation3d(0, 0, robotPose2d.getRotation().getRadians()))
        .plus(turretTranslation);
    Translation3d targetLoc = getTarget();
    


    Rotation2d targetYaw =  
        targetLoc
        .minus(robotPose.getTranslation()).toTranslation2d().getAngle();

    Pose3d targetPose = new Pose3d(targetLoc, new Rotation3d(0,0,targetYaw.getRadians()));
    //System.out.println(targetLoc.minus(robotPose.getTranslation()).toTranslation2d().getAngle().getDegrees() +" "+
    //                  robotPose.getRotation().toRotation2d().getDegrees());
    return new Transform3d(robotPose, targetPose);
  }


  private Translation3d getTarget(){
    if (DriverStation.getAlliance().isEmpty()) 
      return new Translation3d(8,4,0);
    if (DriverStation.getAlliance().get() == Alliance.Red){
      return new Translation3d(13,4,2);
    } else {
      return new Translation3d(4.5,4,2);
    }
  }

  public static record DistanceSample(
    double power,
    Angle hoodAngle,

    Distance distance){
      
  }
  public static record TurretCommand(
    Angle turretAngle,
    double power,
    Angle hoodAngle){
      
  }
}
  
