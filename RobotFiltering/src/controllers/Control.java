package controllers;

import java.util.Vector;

import Jama.Matrix;

public interface Control {
	void startProcess();
	void inputState(Vector<Vector<Double>> State);
	void inputState(Matrix State);
	Boolean finishedProcess();
}
