package controllers;

import java.util.Vector;
//import javax.vecmath.GMatrix;

public interface EKFControl extends Control {
	Vector<Double> getQ(); //Three Double Vector - one Q for each direction
	Vector<Double> getR(); //Three Double Vector - one R for each direction
	Vector<Double> getAlpha();
	//void inputState(GMatrix State); // 3x3!
}
