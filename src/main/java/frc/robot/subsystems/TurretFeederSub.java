package frc.robot.subsystems;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.opencv.core.Mat.Tuple2;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.TurretConstants;

public class TurretFeederSub extends SubsystemBase {
  private TalonFX motor1 = new TalonFX(TurretConstants.FEEDER_MOTOR_1_CANID);
  private TalonFX motor2 = new TalonFX(TurretConstants.FEEDER_MOTOR_2_CANID);

  private final VelocityVoltage feederController1;
  private final VelocityVoltage feederController2;
  
  private final double ratio = 5;
  private final double wheelRad = 2*25.4/1000; //m

  private final TalonFXSimState motor1Sim;
  private final TalonFXSimState motor2Sim;
  private double simSpeed = 0;
  private double simSpeedTarget = 0;
  private final double simAccel = 0.5;



  public TurretFeederSub(){
    TalonFXConfiguration motorConfig = new TalonFXConfiguration();
    motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    motorConfig.Feedback.SensorToMechanismRatio = 1;
    motorConfig.Slot0.kP = 0.1;
    motorConfig.Slot0.kI = 0;
    motorConfig.Slot0.kD = 0;
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    motor1.getConfigurator().apply(motorConfig);
    motor2.getConfigurator().apply(motorConfig);

    feederController1 = new VelocityVoltage(0).withSlot(0);
    feederController2 = new VelocityVoltage(0).withSlot(0);

    
    motor1Sim = motor1.getSimState();
    motor2Sim = motor2.getSimState();
  }


  public void setSpeed(LinearVelocity speed){
    AngularVelocity aSpeed = RotationsPerSecond.of(
        speed.in(MetersPerSecond) * ratio / (2*Math.PI*wheelRad));
    
    motor1.setControl(
        feederController1.withVelocity(aSpeed));
    motor2.setControl(
        feederController2.withVelocity(aSpeed.times(-1)));
    
    if (Robot.isSimulation())
      simSpeedTarget = aSpeed.in(RotationsPerSecond);
  }

  public Tuple2<LinearVelocity> getSpeed(){
    AngularVelocity aSpeed = motor1.getVelocity().getValue();
    return new Tuple2<LinearVelocity>(
        MetersPerSecond.of(
        aSpeed.in(RotationsPerSecond) / ratio * (2*Math.PI*wheelRad)),
        MetersPerSecond.of(
        aSpeed.in(RotationsPerSecond) / ratio * (2*Math.PI*wheelRad))
      );

  }
  
  @Override
  public void periodic() {
    Tuple2<LinearVelocity> speeds = getSpeed();
    SmartDashboard.putNumber("TurretFeeder/motor1Speed", speeds.get_0().in(MetersPerSecond));
    SmartDashboard.putNumber("TurretFeeder/motor2Speed", speeds.get_1().in(MetersPerSecond));
  }


  @Override
  public void simulationPeriodic() {
    if (simSpeed == simSpeedTarget){

    } else if (simSpeed > simSpeedTarget){
      simSpeed -= simAccel;
    } else {
      simSpeed += simAccel;
    }
    motor1Sim.setRotorVelocity(simSpeed);
    motor2Sim.setRotorVelocity(-simSpeed);
  }
}
