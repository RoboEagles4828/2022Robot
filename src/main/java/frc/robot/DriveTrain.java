package frc.robot;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.*;
import frc.robot.Constants.*;

public class DriveTrain {
    public MotorControllerGroup left;
    public MotorControllerGroup right;
    private DifferentialDrive dt;

    public WPI_TalonFX front_left = new WPI_TalonFX(Ports.front_left_port);
    public WPI_TalonFX back_left = new WPI_TalonFX(Ports.back_left_port);
    public WPI_TalonFX front_right = new WPI_TalonFX(Ports.front_right_port);
    public WPI_TalonFX back_right = new WPI_TalonFX(Ports.back_right_port);

    public DriveTrain(){
        left = new MotorControllerGroup(front_left, back_left);
        right = new MotorControllerGroup(front_right, back_right);
        dt = new DifferentialDrive(left, right);
        front_left.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        front_right.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor);
        front_left.setNeutralMode(NeutralMode.Brake);
        front_right.setNeutralMode(NeutralMode.Brake);
        back_left.setNeutralMode(NeutralMode.Brake);
        back_right.setNeutralMode(NeutralMode.Brake);
        left.setInverted(false);
        right.setInverted(true);
    }

    public void set_speeds(double xSpeed, double zRotation){
        dt.arcadeDrive(xSpeed, zRotation);
    }

    public double get_raw_speeds(double speed){
        int pos_neg = 1;
        if(speed<0){
            pos_neg=-1;
            speed=speed*-1;
        }
        if(speed<0.05){return 0;}
        int x = (int)(0.4/speed);
        x++;
        return speed*x*pos_neg;
    }
    

    public void stop(){
        dt.arcadeDrive(0, 0);
    }

    public double convertMeters(double sensorCounts){
        double motorRotations = (double)sensorCounts / Distances.revs_count;
        double wheelRotations = motorRotations / Distances.gear_ratio;
        double positionMeters = wheelRotations * (2 * Math.PI * Units.inchesToMeters(Distances.wheel_size));
        return positionMeters;
    }

    public double convertMeters2(double sensorCount){
        return Units.inchesToMeters(sensorCount/Distances.encoder_ratio);
    }

    public double getVel(){
        double mps = convertMeters(front_left.getSelectedSensorVelocity()) * 10.0;
        return mps;
    }

    public double getVel2(){
        double mps = Units.inchesToMeters(front_left.getSelectedSensorVelocity()/Distances.encoder_ratio)*10;
        return mps;
    }
}
