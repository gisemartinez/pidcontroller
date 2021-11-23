import processing.javafx.*;

/********************************************************
* Arduino PID Tuning Front-End,  Version 0.2
* original by Brett Beauregard, May 2009
* adapted for @me as part of a learning process
* 
* "This application is designed to interface with an
* arduino running the PID Library.  From this Control
* Panel you can observe & adjust PID performance in 
* real time
*
* The ControlP5 library is required to run this sketch.
* files and install instructions can be found at
* http://www.sojamo.de/libraries/controlP5/ "
* 
* Mentioned Arduino and PID library are part of a Proteus schema excluded from this repo.
********************************************************/

import java.nio.ByteBuffer;
import processing.serial.*;
import controlP5.*;

/***********************************************
* User spcification section
**********************************************/
// set the size of the form
int windowWidth = 800;
int windowHeight = 600;

// in = Y-axis; out = X-axis
// set Min and Max for both the top and bottom trends
float inScaleMin = 0;
float inScaleMax = 25;
float outScaleMin = 0;
float outScaleMax = 25;

// number of mS into the past you want to display
int windowSpan = 300000;

// how often you want the graph to be reDrawn;
int refreshRate = 100;

// to display Time as Minutes
float displayFactor = 60000;

// if you'd like to output data to a file, specify the path here
String outputFileName = "";

int nextRefresh;
int arrayLength = windowSpan / refreshRate + 1;
//we might not need them this big, but this is covering the worst case
int[] InputData = new int[arrayLength];
int[] SetpointData = new int[arrayLength];
int[] OutputData = new int[arrayLength];


float inputTop = 25;
float inputHeight = (windowHeight - 70) * 2 / 3;
float outputTop = inputHeight + 50;
float outputHeight = (windowHeight - 70) * 1 / 3;

float ioLeft = 150, ioWidth = windowWidth - ioLeft - 50;
float ioRight = ioLeft + ioWidth;
float pointWidth = (ioWidth) / float(arrayLength - 1);

int vertCount = 10;

int nPoints = 0;

float Input = 0, Setpoint = 0, Output = 0;

boolean madeContact = false;
boolean justSent = true;
boolean didSetup = false;

Serial serialPort;

ControlP5 controlP5;
controlP5.Button AMButton;
controlP5.Textlabel AMLabel, AMCurrent, InLabel, 
OutLabel, SPLabel, PLabel, 
ILabel, DLabel;
controlP5.Textfield SPField, InField, OutField, 
PField, IField, DField;

PrintWriter output;
PFont AxisFont, TitleFont;
/***********************************************
* end user spec
**********************************************/

void setup()
{
    frameRate(30);
    size(800 , 600);
    String serialList = Serial.list();
    println("Available serial ports: " + serialList);
    
    // Communication with simmulated Arduino
    // Serial(parent, portName, baudRate)
    serialPort = new Serial(this, serialList[0], 9600);
    serialPort.bufferUntil(10);
    
    // Initialize the various Buttons, Labels, and Text Fields we'll be using 
    controlP5 = new ControlP5(this);
    // controlP5 legacy addTextfield(java.lang.String theName,int theX,int theY,int theW,int theH)
    SPField = controlP5.addTextfield("Setpoint",10,100,60,20);
    InField = controlP5.addTextfield("Input",10,150,60,20);
    OutField = controlP5.addTextfield("Output",10,200,60,20);
    PField = controlP5.addTextfield("P_Param",10,275,60,20);
    IField = controlP5.addTextfield("I_Param",10,325,60,20);
    DField = controlP5.addTextfield("D_Param",10,375,60,20);
    // controlP5 legacy (deprecated) (java.lang.String theName, float theValue, int theX, int theY, int theW, int theH)
    AMButton = controlP5.addButton("Toggle_AM",0.0,10,50,60,20);
    // controlP5 legacy addTextlabel(java.lang.String theName, java.lang.String theText, int theX, int theY)
    AMLabel = controlP5.addTextlabel("AM","Manual",12,72);
    AMCurrent = controlP5.addTextlabel("AMCurrent","Manual",80,65);
    controlP5.addButton("Send_To_Arduino",0.0,10,425,120,20);
    SPLabel = controlP5.addTextlabel("SP","3",80,103);
    InLabel = controlP5.addTextlabel("In","1",80,153);
    OutLabel = controlP5.addTextlabel("Out","2",80,203);
    PLabel = controlP5.addTextlabel("P","4",80,278);
    ILabel = controlP5.addTextlabel("I","5",80,328);
    DLabel = controlP5.addTextlabel("D","6",80,378);
    
    AxisFont = loadFont("axis.vlw");
    TitleFont = loadFont("Titles.vlw");
    
    nextRefresh = millis();
    if (outputFileName!= "") output = createWriter(outputFileName);
    
    didSetup = true;
    println("setup!");
}

void draw()
{
    background(200);
    drawGraph();
    drawButtonArea();
}

void drawGraph()
{
    if (!madeContact) return;
    
    //draw Base, gridlines
    stroke(0);
    fill(230);
    rect(ioLeft, inputTop,ioWidth - 1 , inputHeight);
    rect(ioLeft, outputTop, ioWidth - 1, outputHeight);
    stroke(210);
    
    //Section Titles
    textFont(TitleFont);
    fill(255);
    text("PID Input / Setpoint",(int)ioLeft + 10,(int)inputTop - 5);
    text("PID Output",(int)ioLeft + 10,(int)outputTop - 5);
    
    
    //GridLines and Titles
    textFont(AxisFont);
    
    //horizontal grid lines
    int interval = (int)inputHeight / 5;
    for (int i = 0; i < 6; i++) {
        if (i > 0 &&  i < 5) line(ioLeft + 1,inputTop + i * interval,ioRight - 2,inputTop + i * interval);
        text(str((inScaleMax - inScaleMin) / 5 * (float)(5 - i) + inScaleMin),ioRight + 5,inputTop + i * interval + 4);
    }
    interval = (int)outputHeight / 5;
    for (int i = 0; i < 6; i++) {
        if (i > 0 &&  i < 5) line(ioLeft + 1,outputTop + i * interval,ioRight - 2,outputTop + i * interval);
        text(str((
            outScaleMax - outScaleMin) / 5 * (float)(5 - i) + outScaleMin),ioRight + 5,outputTop + i * interval + 4);
    }
    
    //vertical grid lines and TimeStamps
    int elapsedTime = millis();
    interval = (int)ioWidth / vertCount;
    int shift = elapsedTime * (int)ioWidth / windowSpan;
    shift %=  interval;
    
    int iTimeInterval = windowSpan / vertCount;
    float firstDisplay = (float)(iTimeInterval * (elapsedTime / iTimeInterval)) / displayFactor;
    float timeInterval = (float)(iTimeInterval) / displayFactor;
    for (int i = 0; i < vertCount; i++) {
        int x = (int)ioRight - shift - 2 - i * interval;
        
        line(x, inputTop + 1, x, inputTop + inputHeight - 1);
        line(x, outputTop + 1, x, outputTop + outputHeight - 1);
        
        float t = firstDisplay - (float)i * timeInterval;
        if (t >=  0)  text(str(t),x,outputTop + outputHeight + 10);
    }
    
    
    // add the latest data to the data Arrays. The values need
    // to be messaged to get them to graph correctly. They 
    // need to be scaled to fit where they're going, and 
    // because 0,0 is the top left, we need to flip the values.
    // This is easier than having the user stand on their head
    // to read the graph.
    if (millis() > nextRefresh && madeContact) {
        nextRefresh += refreshRate;
        
        for (int i = nPoints - 1; i > 0; i--) {
            InputData[i] = InputData[i - 1];
            SetpointData[i] = SetpointData[i - 1];
            OutputData[i] = OutputData[i - 1];
        }
        if (nPoints < arrayLength) nPoints++;
        
        InputData[0] = int(inputHeight) - int(inputHeight * (Input - inScaleMin) / (inScaleMax - inScaleMin));
        SetpointData[0] = int(inputHeight) - int(inputHeight * (Setpoint - inScaleMin) / (inScaleMax - inScaleMin));
        OutputData[0] = int(outputHeight) - int(outputHeight * (Output - outScaleMin) / (
            outScaleMax - outScaleMin));
    }
    //draw lines for the input, setpoint, and output
    strokeWeight(2);
    for (int i = 0; i < nPoints - 2; i++)
        {
        int X1 = int(ioRight - 2 - float(i) * pointWidth);
        int X2 = int(ioRight - 2 - float(i + 1) * pointWidth);
        boolean y1Above,y1Below, y2Above, y2Below;
        
        
        //DRAW THE INPUT
        boolean drawLine = true;
        stroke(255,0,0);
        drawInputValues(InputData);
        
        //DRAW THE SETPOINT
        drawLine = true;
        stroke(0,255,0);
        drawInputValues(SetpointData);
        
        //DRAW THE OUTPUT
        drawLine = true;
        stroke(0,0,255);
        Y1 = OutputData[i];
        Y2 = OutputData[i + 1];
        
        y1Above = (Y1 > outputHeight);
        y1Below = (Y1 < 0);
        y2Above = (Y2 > outputHeight);
        y2Below = (Y2 < 0);
        // if both points are outside the min or max, don't draw the line. If only one 
        // point is outside constrain it to the limit, and leave the other one untouched.
        if (y1Above) {
            if (y2Above) drawLine = false;
            else if (y2Below) {
                Y1 = (int)outputHeight;
                Y2 = 0;
            } else Y1 = (int)outputHeight;
        } else if (y1Below) {
            if (y2Below) drawLine = false;
            else if (y2Above) {
                Y1 = 0;
                Y2 = (int)outputHeight;
            } else Y1 = 0;
        } else {
            if (y2Below) Y2 = 0;
            else if (y2Above) Y2 = (int)outputHeight;
        }
        
        if (drawLine) {
            line(X1, outputTop + Y1, X2, outputTop + Y2);
        }
    }
    strokeWeight(1);
}

void drawInputValues(int[] InputData){
    int Y1 = InputData[i];
    int Y2 = InputData[i + 1];
    
    y1Above = (Y1 > (int)inputHeight);
    y1Below = (Y1 < 0);
    y2Above = (Y2 > (int)inputHeight);
    y2Below = (Y2 < 0);
    // if both points are outside the min or max, don't draw the line. If only one 
    // point is outside constrain it to the limit, and leave the other one untouched.
    if (y1Above) {
        if (y2Above) drawLine = false;
        else if (y2Below) {
            Y1 = (int)inputHeight;
            Y2 = 0;
        }
        else Y1 = (int)inputHeight;
    } else if (y1Below) {
        if (y2Below) drawLine = false;
        else if (y2Above) {
            Y1 = 0;
            Y2 = (int)inputHeight;
        }
        else Y1 = 0;
    } else {
        if (y2Below) Y2 = 0;
        else if (y2Above) Y2 = (int)inputHeight;
    }
    
    if (drawLine) {
        line(X1, Y1 + inputTop, X2, Y2 + inputTop);
    }
}

void drawButtonArea()
    {
    stroke(0);
    fill(100);
    rect(0, 0, ioLeft, windowHeight);
}

String  mode = "Manual";
void Toggle_AM() {
    if (mode ==  "Manual") {
        mode = "Automatic";
        AMLabel.setValue(mode);
    } else {
        mode = "Manual";
        AMLabel.setValue(mode);
    }
}

// Sending Floating point values to the arduino
// is a huge pain. if anyone knows an easier
// way please let know. the way I'm doing it:
// - Take the 6 floats we need to send and
//   put them in a 6 member float array.
// - using the java ByteBuffer class, convert
//   that array to a 24 member byte array
// - send those bytes to the arduino
void Send_To_Arduino()
    {
    float[] toSend = new float[6];
    
    toSend[0] = float(SPField.getText());
    toSend[1] = float(InField.getText());
    toSend[2] = float(OutField.getText());
    toSend[3] = float(PField.getText());
    toSend[4] = float(IField.getText());
    toSend[5] = float(DField.getText());
    Byte b = (mode ==  "Manual") ? (byte)0 : (byte)1;
    serialPort.write(b);
    serialPort.write(floatArrayToByteArray(toSend));
    justSent = true;
} 


byte[] floatArrayToByteArray(float[]input)
    {
    int len = 4 * input.length;
    int index = 0;
    byte[] b = new byte[4];
    byte[] out = new byte[len];
    ByteBuffer buf = ByteBuffer.wrap(b);
    for (int i = 0;i < input.length;i++) {
        buf.position(0);
        buf.putFloat(input[i]);
        for (int j = 0;j < 4;j++) out[j + i * 4] = b[3 - j];
    }
    return out;
}


// take the string the arduino sends us and parse it
void serialEvent(Serial serialPort)
    {
    String read = serialPort.readStringUntil(10);
    println("got : " + read);
    if (outputFileName!= "") output.print(str(millis()) + " " + read);
    String[] s = split(read, " ");
    
    if (!didSetup) {
        println("Setup wasn't completed yet");
        return;
    }

    if (s.length ==  8) {
        // pull the information we need out of the string and put it where it's needed
        Setpoint = float(s[1]);
        Input = float(s[2]);
        Output = float(s[3]);
        SPLabel.setValue(s[1]);
        InLabel.setValue(s[2]);
        OutLabel.setValue(trim(s[3]));
        PLabel.setValue(trim(s[4]));
        ILabel.setValue(trim(s[5]));
        DLabel.setValue(trim(s[6]));
        AMCurrent.setValue(trim(s[7]));
        
        // if this is the first read since we sent values to the arduino,
        // take the current values and put them into the input fields
        if (justSent) {
            SPField.setText(trim(s[1]));
            InField.setText(trim(s[2]));
            OutField.setText(trim(s[3]));
            PField.setText(trim(s[4]));
            IField.setText(trim(s[5]));
            DField.setText(trim(s[6]));
            mode = trim(s[7]);
            AMLabel.setValue(mode);
            justSent = false;
        }
        
        if (!madeContact) madeContact = true;
    }
}
