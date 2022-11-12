// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class Intake extends SubsystemBase {

private double flywheelTolerance = 0.05; // Tolerance of PID controller
private boolean override = false; // Helps us switch from manual to auto
private Timer overrideTimer = new Timer(); // We want a toggle button of some sorts
private double overrideTime = 1.0;

private final WPI_TalonSRX intakeFlyWheel = new WPI_TalonSRX(Constants.ShooterPorts.LeftFlywheelPort);


private final PIDController intakeFlyWheelPID = new  PIDController(Constants.leftFlywheelPIDConsts.pidP, Constants.leftFlywheelPIDConsts.pidI, Constants.leftFlywheelPIDConsts.pidD);

private SimpleMotorFeedforward intakeFlyWheelFF = new  SimpleMotorFeedforward(Constants.leftFlywheelFF.kS, Constants.leftFlywheelFF.kV, Constants.leftFlywheelFF.kA);



  public Intake() {
    intakeFlyWheel.configFactoryDefault();
    intakeFlyWheel.setInverted(true); 
    intakeFlyWheel.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute);

   
    intakeFlyWheelPID.setTolerance(flywheelTolerance);

    overrideTimer.start(); // Start timer
    overrideTimer.reset(); // Reset timer

  }
  public void setFlywheelPower(double speed) {
    intakeFlyWheel.set(speed);
    }
    public boolean flywheelWithinErrorMargin() {
    return (intakeFlyWheelPID.atSetpoint());
    }
    public void setFlywheelConstantVelocity(double RPM) {
    intakeFlyWheel.setVoltage((intakeFlyWheelFF.calculate(RPM/60.0)) + intakeFlyWheelPID.calculate(getLeftRPM(), RPM));
    }
    public double getLeftRPM() {
      return ((intakeFlyWheel.getSelectedSensorVelocity() * 10)/4096.0)*60.0;
      }
      public double getRightRPM() {
      return ((intakeFlyWheel.getSelectedSensorVelocity() * 10)/4096.0)*60.0;
      }
      public double getLeftFlywheelPower() {
      return intakeFlyWheel.get();
      }
      public double getRightFlywheelPower() {
      return intakeFlyWheel.get();
      }
      public double getCurrent(){
        return intakeFlyWheel.getStatorCurrent();
      }
      public double getAverageRPM() {
        return ((getLeftRPM()/2.0));
        }
        public double getFlywheelCurrent() {
        return (intakeFlyWheel.getStatorCurrent()/2.0);
        
        }
       
        public void resetFlywheelEncoders() {
        intakeFlyWheel.setSelectedSensorPosition(0, 0, 10);
        }
        

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
SmartDashboard.putNumber("Average RPM", getAverageRPM());
SmartDashboard.putNumber("Average Current", getFlywheelCurrent());
SmartDashboard.putNumber("Left Flywheel RPM", getLeftRPM());
SmartDashboard.putNumber("Left Flywheel Power", getLeftFlywheelPower());
SmartDashboard.putNumber("Right Flywheel RPM", getRightRPM());
SmartDashboard.putNumber("Right Flywheel Power", getRightFlywheelPower());
SmartDashboard.putNumber("Intake Flywheel Current", getCurrent());

if (RobotContainer.getJoy1().getRawButton(2) && overrideTimer.get() >=  overrideTime) {
  override = !override;
  overrideTimer.reset();
  }
  if (override) { // Auto code
    if (RobotContainer.getJoy1().getRawButton(1)) {
    setFlywheelConstantVelocity(1000.0); // Sets it to 1000 RPM
    } else {
    setFlywheelConstantVelocity(0.0);
    setFlywheelPower(0.0);
    }
    } else if (!override) { // Default manual override
    setFlywheelPower(-1.0*RobotContainer.getJoy1().getY());
    }
      
  }
}

