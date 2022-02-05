package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

public class Conveyor {
    private WPI_TalonFX conveyor;

    public Conveyor(){
        conveyor = new WPI_TalonFX(Constants.conveyor_port);
    }

    public void set_speed(double speed){
        conveyor.set(speed);
    }
}
