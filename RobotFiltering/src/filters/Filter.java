package filters;

import java.util.Vector;

import Jama.Matrix;

import testsystem.Data;
import testsystem.GPSEntry;

import controllers.Control;

public interface Filter {
	void setData(Data Dataset);
	Boolean finishedProcess();
	void startProcess();
	void setControl(Control Controller);
	void outputData(String FileName);
	long getTimeElapsed();
	Vector<GPSEntry> getOutput();
	Vector<Matrix> getResiduals();
}
