package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.*;
import frc.robot.Constants.*;

public class DriveTrain {
    private MotorControllerGroup left;
    private MotorControllerGroup right;
    private DifferentialDrive dt;
    public WPI_TalonFX front_left = new WPI_TalonFX(Ports.front_left_port);
    public WPI_TalonFX back_left = new WPI_TalonFX(Ports.back_left_port);
    public WPI_TalonFX front_right = new WPI_TalonFX(Ports.front_right_port);
    public WPI_TalonFX back_right = new WPI_TalonFX(Ports.back_right_port);
    public DriveTrain(){
        left = new MotorControllerGroup(front_left, back_left);
        right = new MotorControllerGroup(front_right, back_right);
        dt = new DifferentialDrive(left, right);

        left.setInverted(false);
        right.setInverted(true);
    }

    public void set_speeds(double xSpeed, double zRotation){
        dt.arcadeDrive(xSpeed, zRotation);
    }

    public void stop(){
        dt.arcadeDrive(0, 0);
    }
}
