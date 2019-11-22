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

    ArtArm(double length1, double length2, double baseGear, double motorGear, double gear1, double gear2){
        l1 = length1;
        l2 = length2;

        gb = baseGear;
        gm = motorGear;

        g1 = gear1;
        g2 = gear2;
    }

    private int position = 99; //current position number
    //99 = home; 10 = extended; 1 = dropped; 100 = scorpion

    private double x = 0; //current x position in inches
    private double y = 0; //current y position in inches

    private double getX(){
        return x;
    }

    private double getY(){
        return y;
    }

    private double getDist(double x, double y){
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
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
        return (int)((getA1()/* - 120*1680/360.0*/)*(gb/gm) * 1680 / 360); //motor counts
    }

    public int getM2(){
        return (int)((getA3()/* - 33.5*1680/360.0*/)*(gb/gm) * 1680 / 360); //motor counts
    }

    public int getEEDeg(){
        if(position == 100)
            return 255;
        else
            return (int)((90 - getD2()) + (180 - getD1() -getA2()));
    }


    public void extendPos(){
        double xe = 0; //extended x position
        double ye = 0; //extended y position

        x = xe;
        y = ye;

        position = 10;
    }

    public void adjust(double adj){
        x += adj;
        if(x > l1+l2-1)
            x = l1+l2-1;
    }

    public void dropPos(){
        y -= 3;
        if(y < 6.7-3.5)
            y = 6.7-3.5;
        position = 1;
    }

    public void scorpionPos(){
        double xs = 0; //scorpion x position
        double ys = 0; //scorpion y position

        x = xs;
        y = ys;

        position = 100;
    }

    public int getPos(){ return position; }

    public void setPos(int pos){
        position = pos;
    }

}
