package controllers;

import java.util.Vector;

import javax.vecmath.GMatrix;

import Jama.Matrix;

public class DoubleFuzzySPKF implements SPKFControl{

	public DoubleFuzzySPKF(String FN) {
		
	}
	
	public Double getAlpha() {
		// TODO Auto-generated method stub
		return null;
	}

	public Double getBeta() {
		// TODO Auto-generated method stub
		return null;
	}

	public Double getGamma() {
		// TODO Auto-generated method stub
		return null;
	}

	public Double getKappa() {
		// TODO Auto-generated method stub
		return null;
	}

	public Double getLambda() {
		// TODO Auto-generated method stub
		return null;
	}

	public Matrix getP() {
		// TODO Auto-generated method stub
		return null;
	}

	public void inputState(Matrix Cov, Matrix Inno) {
		// TODO Auto-generated method stub
		
	}

	public void setAlpha(Double Lam) {
		// TODO Auto-generated method stub
		
	}

	public void setBeta(Double Bet) {
		// TODO Auto-generated method stub
		
	}

	public void setKappa(Double Kap) {
		// TODO Auto-generated method stub
		
	}

	public void setL(int L) {
		// TODO Auto-generated method stub
		
	}

	public Boolean finishedProcess() {
		// TODO Auto-generated method stub
		return null;
	}

	public void inputState(Vector<Vector<Double>> State) {
		// TODO Auto-generated method stub
		
	}

	public void inputState(Matrix State) {
		// TODO Auto-generated method stub
		
	}

	public void startProcess() {
		// TODO Auto-generated method stub
		
	}

	public int getControlType() {
		// TODO Auto-generated method stub
		return 0;
	}



}
