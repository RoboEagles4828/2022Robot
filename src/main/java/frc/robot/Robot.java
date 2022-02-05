// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.*;
import edu.wpi.first.wpilibj.*;

public class Robot extends TimedRobot {
  DriveTrain dt = new DriveTrain(new WPI_TalonFX(Constants.front_left_port), new WPI_TalonFX(Constants.back_left_port),
                       new WPI_TalonFX(Constants.front_right_port), new WPI_TalonFX(Constants.back_right_port));

  Shooter shooter = new Shooter();
  boolean manual_shooter = false;

  Conveyor conveyor = new Conveyor();
  boolean manual_conveyor = false;

  Intake intake = new Intake();
  boolean manual_intake = false;

  DigitalInput prox_lower = new DigitalInput(Constants.proxy_lower);
  DigitalInput prox_upper = new DigitalInput(Constants.proxy_upper);

  Joystick joystick_0 = new Joystick(Constants.joystick_0_port);
  Joystick joystick_1 = new Joystick(Constants.joystick_1_port);

  @Override
  public void teleopInit() {
    /* factory default values */
    /*_talonL.configFactoryDefault();
    _talonR.configFactoryDefault();*/

    /* flip values so robot moves forward when stick-forward/LEDs-green */
    /*_talonL.setInverted(false); // <<<<<< Adjust this
    _talonR.setInverted(false); // <<<<<< Adjust this*/
    /*
     * WPI drivetrain classes defaultly assume left and right are opposite. call
     * this so we can apply + to both sides when moving forward. DO NOT CHANGE
     */
  }

  @Override
  public void teleopPeriodic() {
    double xSpeed = joystick_0.getRawAxis(1) * -1; // make forward stick positive
    double zRotation = joystick_0.getRawAxis(2) * -1; // WPI Drivetrain uses positive=> right

    dt.set_speeds(xSpeed, zRotation);

    //If ball detected make conveyor move to pull it
    if(prox_lower.get()){
      conveyor.set_speed(Constants.conveyor_speed);
    }else if(!manual_conveyor){//If ball is not detected, so long as it is not under manual control conveyor will stop
      conveyor.set_speed(0);
    }

    //If ball is detected by upper proxy stop conveyor
    if(prox_upper.get()&&!manual_shooter){
      conveyor.set_speed(0);
    }

    //Manually control shooter
    if (joystick_0.getRawButton(1)){
      shooter.set_speed(Constants.shooter_speed);
      manual_shooter = true;
    }

    //Stop shooter when not manually controlled
    if (joystick_0.getRawButtonReleased(1)){
      shooter.set_speed(0);
      manual_shooter = false;
    }
    
    //Manually control intake
    if (joystick_0.getRawButton(2)){
      intake.set_speed(Constants.intake_speed);
      manual_intake = true;
    }

    //Stop intake when not manually controlled
    if (joystick_0.getRawButtonReleased(2)){
      intake.set_speed(0);
      manual_intake = false;
    }

    //Manually control conveyor
    if (joystick_0.getRawButton(3)){
      conveyor.set_speed(Constants.conveyor_speed);
      manual_conveyor = true;
    }

    //Stop conveyor when not manually controlled
    if (joystick_0.getRawButtonReleased(3)){
      manual_conveyor = false;
      conveyor.set_speed(0);
    }
  }
}