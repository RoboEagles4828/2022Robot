// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.CounterBase;

public class Robot extends TimedRobot {

  DriveTrain dt = new DriveTrain();
  
  //ErrorCode error =  front_left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);
  
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

  static final String DefaultAuto = "Default";
  static final String CustomAuto = "Drive, Intake, Shoot";
  SendableChooser<String> chooser = new SendableChooser<>();
  String autoSelected;
  Timer timer = new Timer();

  double starting_pos = 0;
  int auto_state = 0;
  double start_time = 0;

  @Override
  public void robotInit(){
    chooser.setDefaultOption("Default Auto", DefaultAuto);
    chooser.addOption("Drive, Intake, Shoot", CustomAuto);
    SmartDashboard.putData("Auto Choices:", chooser);
    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();//Call twice to automatically create both cameras and have them as optional displays
    //dt.front_left.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative);//Defaults to integrated sensor, this is quadrature
  }

  @Override
  public void autonomousInit(){
    autoSelected=chooser.getSelected();
    System.out.println("Auto selected: " + autoSelected);
    timer.reset();
    timer.start();
    starting_pos=dt.front_left.getSelectedSensorPosition();
  }

  @Override
  public void autonomousPeriodic(){
    //If ball detected make conveyor move to pull it
    if(prox_lower.get()){
      conveyor.set_speed(Constants.index_conveyor_speed);
    }else{//If ball is not detected, conveyor will stop
      conveyor.stop();
    }

    //If ball is detected by upper proxy stop conveyor
    if(prox_upper.get()){
      conveyor.stop();
    }

    //Determine what auto to run
    switch(autoSelected){
      case CustomAuto:
          switch(auto_state){
            case 0:
              if(Autonomous.intake_drive(Constants.wall_to_ball, intake, dt, starting_pos, 1)){
                starting_pos=dt.front_left.getSelectedSensorPosition();
                auto_state++;
              }
              break;
            case 1:
              if(Autonomous.drive(Constants.wall_to_ball, dt, starting_pos, -1)){
                starting_pos=dt.front_left.getSelectedSensorPosition();
                start_time=timer.get();
                auto_state++;
              }
              break;
            case 2:
              if(Autonomous.shoot(Constants.wall_shoot_time, start_time, timer.get(), shooter)){auto_state++;}
              break;
            default:
              break;
          }
        break;
      case DefaultAuto://Start flush with wall and just shoot
        if(Autonomous.shoot(Constants.wall_shoot_time, 0, timer.get(), shooter)){break;}
        break;
      default:
        if(Autonomous.shoot(Constants.wall_shoot_time, 0, timer.get(), shooter)){break;}
        break;
    }
  }
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
    double zRotation = joystick_0.getRawAxis(2); // WPI Drivetrain uses positive=> right

    dt.set_speeds(xSpeed, zRotation);
    System.out.println(dt.front_left.getSelectedSensorPosition());
    //If ball detected make conveyor move to pull it
    /*if(prox_lower.get()){
      conveyor.set_speed(Constants.index_conveyor_speed);
    }else if(!manual_conveyor){//If ball is not detected, so long as it is not under manual control conveyor will stop
      conveyor.stop();
    }

    //If ball is detected by upper proxy stop conveyor
    if(prox_upper.get()&&!manual_shooter){
      conveyor.stop();
    }*/

    //Manually control shooter
    if (joystick_0.getRawButton(Constants.manual_shoot_button)){
      shooter.set_speed(Constants.manual_shooter_speed);
      manual_shooter = true;
    }

    //Stop shooter when not manually controlled
    if (joystick_0.getRawButtonReleased(Constants.manual_shoot_button)){
      shooter.stop();
      manual_shooter = false;
    }
    
    //Manually control intake
    if (joystick_0.getRawButton(Constants.manual_intake_button)){
      intake.set_speed(Constants.manual_intake_speed);
      manual_intake = true;
    }

    //Stop intake when not manually controlled
    if (joystick_0.getRawButtonReleased(Constants.manual_intake_button)){
      intake.stop();
      manual_intake = false;
    }

    //Manually control conveyor
    if (joystick_0.getRawButton(Constants.manual_conveyor_button)){
      conveyor.set_speed(Constants.manual_conveyor_speed);
      manual_conveyor = true;
    }

    //Stop conveyor when not manually controlled
    if (joystick_0.getRawButtonReleased(Constants.manual_conveyor_button)){
      manual_conveyor = false;
      conveyor.stop();
    }

    //Manually control conveyor backwards
    if (joystick_0.getRawButton(Constants.manual_conveyor_back_button)){
      conveyor.set_speed(Constants.manual_conveyor_back_speed);
      manual_conveyor = true;
    }

    //Stop conveyor when not manually controlled
    if (joystick_0.getRawButtonReleased(Constants.manual_conveyor_back_button)){
      manual_conveyor = false;
      conveyor.stop();
    }
  }
}