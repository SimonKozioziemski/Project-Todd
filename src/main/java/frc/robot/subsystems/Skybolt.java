package frc.robot.subsystems;

public class Skybolt {
    Jumper flyfly;
    public Skybolt(Jumper flyfly){
        this.flyfly = flyfly;
    }
    public boolean dunk(){
        if(flyfly.whereIsFlyFly().getZ()>2){
            return true;
        }
        return false;
    }
}
