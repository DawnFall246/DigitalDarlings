/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

/**
 * This a class that does calculations and ultimately returns the counts/degrees of the base motors
 * and end effector servo joint
 */


public class ArtArm {
    private double l1, l2; //lengths of arm segments
    private double g1, g2; //chain gears
    private double gb, gm; // big gear and motor gear
    private double x, y;
    private ArmHardware robot;

    ArtArm(ArmHardware Robot, double length1, double length2, double baseGear, double motorGear, double gear1, double gear2) {
        //robot = Robot;

        l1 = length1;
        l2 = length2;

        gb = baseGear;
        gm = motorGear;

        g1 = gear1;
        g2 = gear2;

        x = 0;
        y = 0;
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    private double getDist(double x, double y){
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
    }

    public double maxDist(){
        return l1 + l2;
    }

    public double maxY(){
        return Math.sqrt(Math.pow(maxDist(), 2) - Math.pow(getX(), 2));
    }

    public double maxX(){
        return Math.sqrt(Math.pow(maxDist(), 2) - Math.pow(getY(), 2));
    }

    private double getD1(){
        return Math.acos((Math.pow(getDist(getX(), getY()), 2)) + Math.pow(l1, 2) - Math.pow(l2, 2) / 2*getDist(getX(), getY())*l1);
    }

    private double getD2(){
        return Math.atan(getY()/getX());
    }

    public double getA1(){
        //(d1 + d2)
        return getD1() + getD2();
    }

    public double getA2(){
        return Math.acos((Math.pow(l2, 2) - Math.pow(l1, 2) + Math.pow(getDist(getX(), getY()), 2)) / 2*l1*l2);
    }

    public double getA3(){
        return (g2/g1)*getA2() - getA1();
    }


    public int getM1(){
        return (int)((getA1()/* - 120*1680/360.0*/)*(gb/gm) * 1680.0 / 360); //motor counts
    }

    public int getM2(){
        return (int)((getA3()/* - 33.5*1680/360.0*/)*(gb/gm) * 1680.0 / 360); //motor counts
    }

    public int getEEDeg(){
        return (int)((180 - getD2()) + (180 - getD1() - getA2()));
    }

    public int[] goToXY(double xcor, double ycor){
        x = xcor;
        y = ycor;

        int[] array = {getM1(), getM2(), getEEDeg()};
        return array;
        //robot.ArmBase.setTargetPosition(getM1());
        //robot.ArmJoint.setTargetPosition(getM2());
        //robot.EndJoint.setPosition(getEEDeg() / 360.0);
    }

}
