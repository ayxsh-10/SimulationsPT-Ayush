// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.sims.mechanisms.Mechanator;
import frc.robot.sims.simulations.ElevatorSimulation;

public class ExampleSubsystem extends SubsystemBase implements Reportable {
  private final TalonFX motor;

  private double desiredPosition = 0.67;
  private boolean enabled = true;
  private MotionMagicVoltage motionMagicVoltage;
  private final NeutralOut neutralRequest = new NeutralOut();
  private final ElevatorSimulation esim;
  public ExampleSubsystem(){
    motor = new TalonFX(0);
    motionMagicVoltage = new MotionMagicVoltage(0);

    setMotorConfigs();

    CommandScheduler.getInstance().registerSubsystem(this);
    MechanismLigament2d ligament = Mechanator.getInstance().getLigament("ayush", 1.5, 0.1, 0, 90);
     esim = new ElevatorSimulation(motor, ligament, 10.0, 0.5, 0.05, 0.1, 1,0);
  }

  public void setMotorConfigs(){
        TalonFXConfiguration config = new TalonFXConfiguration();

        // config.Feedback. // ratios
        // config.Slot0. // kP, kI, kD
        // config.MotionMagic. // kP, kI, kD
        config.Feedback.SensorToMechanismRatio = 1.0;
        config.Feedback.RotorToSensorRatio = 1.0;

        config.Slot0.kP = 16.0; // kP, kI, kD
        config.Slot0.kI = 0.0;
        config.Slot0.kD = 0.0;
        config.MotionMagic.MotionMagicCruiseVelocity = 150.0;
        config.MotionMagic.MotionMagicAcceleration = 1500.0;
        config.MotionMagic.MotionMagicJerk = 15000.0;
        
        StatusCode statusCode = motor.getConfigurator().apply(config);
        if (!statusCode.isOK()){
            DriverStation.reportError("Could not apply motor configs, " + statusCode.getDescription(), true);
        }
  }

  @Override
  public void periodic() {
    if (!enabled) {
      return;
    }

    motor.setControl(motionMagicVoltage.withPosition(desiredPosition));
  }


  public void setEnabled(boolean enabled){
    this.enabled = enabled;
    if(!enabled){
      motor.setControl(neutralRequest);
    } 
  }

  public void setTargetPosition(double position){
    desiredPosition = position;
  }

  public double getPosition(){
    return motor.getPosition().getValueAsDouble();
  }

  public double getTargetPosition(){
    return desiredPosition;
  }

  public boolean atSpeed(){
    return getPosition() > getTargetPosition();
  }

  @Override
  public void initShuffleboard(LOG_LEVEL priority) {}
}
