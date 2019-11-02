// assign2.cpp : Defines the entry point for the console application.
//

/*
	CSCI 480 Computer Graphics
	Assignment 2: Simulating a Roller Coaster
	C++ starter code
*/

#include "stdafx.h"
#include <windows.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <iostream>
#include <GL/glu.h>
#include <GL/glut.h>
#include "Camera.h"

#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"

#ifndef GL_CLAMP_TO_EDGE
#define GL_CLAMP_TO_EDGE 0x812F
#endif

int g_iMenuId;

int g_vMousePos[2] = {0, 0};
int g_iLeftMouseButton = 0;    /* 1 if pressed, 0 if not */
int g_iMiddleMouseButton = 0;
int g_iRightMouseButton = 0;

/* state of the world */
float g_vLandRotate[3] = {0.0, 0.0, 0.0};
float g_vLandTranslate[3] = {0.0, 0.0, 0.0};
float g_vLandScale[3] = {1.0, 1.0, 1.0};




// sky
unsigned int skybox[6];
enum {SKY_LEFT,SKY_BACK,SKY_RIGHT,SKY_FRONT,SKY_TOP,SKY_BOTTOM};

unsigned int costerTex;

/* Object where you can load an image */
cv::Mat3b imageBGR;
bool drawing = false;

/* represents one control point along the spline */
struct point {
	double x;
	double y;
	double z;

	point()
	{
		x = 0;
		y = 0;
		z = 0;
	}

	point(double a, double b, double c)
	{
		x = a;
		y = b;
		z = c;
	}

	void crossProduct(point a, point b)
	{
		x = a.y * b.z - b.y * a.z;
		y = b.x * a.z - a.x * b.z;
		z = a.x * b.y - b.x * a.y;
	}

	void normalize()
	{
		double mag = sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
		if (mag != 0)
		{
			x /= mag;
			y /= mag;
			z /= mag;
		}
	}

	void derivative(double u, point a, point b, point c, point d)
	{
		double sqr = u * u;
		x = 0.5 * (a.x * (-3 * sqr + 4 * u - 1) 
			+ b.x * (9 * sqr - 10 * u)
			+ c.x * (-9 * sqr + 8 * u + 1)
			+ d.x * (3 * sqr - 2 * u));

		y = 0.5 * (a.y * (-3 * sqr + 4 * u - 1)
			+ b.y * (9 * sqr - 10 * u)
			+ c.y * (-9 * sqr + 8 * u + 1)
			+ d.y * (3 * sqr - 2 * u));

		z = 0.5 * (a.z * (-3 * sqr + 4 * u - 1)
			+ b.z * (9 * sqr - 10 * u)
			+ c.z * (-9 * sqr + 8 * u + 1)
			+ d.z * (3 * sqr - 2 * u));

	}

	point operator*(const double a)
	{
		return {a * x, a * y, a * z};
	}

	point operator+(const point p)
	{
		return { x + p.x, y + p.y, z + p.z };
	}

	point operator-(const point p)
	{
		return { x - p.x, y - p.y, z - p.z };
	}

	void tangent(double u, point a, point b, point c, point d)
	{
		derivative(u, a, b, c, d);
		normalize();
	}

	void normal(point nt, point nb, bool b)
	{
		if (b) {
			crossProduct(nt, point(0, 1, 0));
		} else {
			crossProduct(nb, nt);
		}
		normalize();
	}

	void binormal(point nt, point nn)
	{
		crossProduct(nt, nn);
		normalize();
	}

	void generatePoints(double u, point a, point b, point c, point d)
	{
		double cube = u * u * u;
		double sqr = u * u;

		x = 0.5 * (a.x * (-1 * cube + 2 * sqr - u)
			+ b.x * (3 * cube - 5 * sqr + 2)
			+ c.x * (-3 * cube + 4 * sqr + u)
			+ d.x * (cube - sqr));

		y = 0.5 * (a.y * (-1 * cube + 2 * sqr - u)
			+ b.y * (3 * cube - 5 * sqr + 2)
			+ c.y * (-3 * cube + 4 * sqr + u)
			+ d.y * (cube - sqr));

		z = 0.5 * (a.z * (-1 * cube + 2 * sqr - u)
			+ b.z * (3 * cube - 5 * sqr + 2)
			+ c.z * (-3 * cube + 4 * sqr + u)
			+ d.z * (cube - sqr));
	}

	double distance(point a)
	{
		return sqrt(pow(x - a.x, 2) + pow(y - a.y, 2) + pow(z - a.z, 2));
	}
};

// Camera 
Camera camera;

int camIndex = 0;
point g_camPos = point();
point g_camForward = point();
point g_camUp = point();
point g_camRight = point(0, 1, 0);


/* spline struct which contains how many control points, and an array of control points */
struct spline {
	int numControlPoints;
	struct point *points;
};

/* the spline array */
struct spline *g_Splines;

/* total number of splines */
int g_iNumOfSplines;

// List of points from spine
std::vector<point> pList;

std::vector<point> tanList;	// tangent list
std::vector<point> norList; // normal list
std::vector<point> biList; 	// binormal list

void bruteForceSplines();

/* Read an image into memory. 
Set argument displayOn to true to make sure images are loaded correctly.
One image loaded, set to false so it doesn't interfere with OpenGL window.*/
int readImage(char *filename, cv::Mat3b& image, bool displayOn)
{
	std::cout << "reading image: " << filename << std::endl;
	image = cv::imread(filename);
	if (!image.data) // Check for invalid input                    
	{
		std::cout << "Could not open or find the image." << std::endl;
		return 1;
	}

	if (displayOn)
	{
		cv::imshow("TestWindow", image);
		cv::waitKey(0); // Press any key to enter. 
	}
	return 0;
}

unsigned char getPixelValue(cv::Mat3b& image, int x, int y, int chan)
{
	return image.at<cv::Vec3b>(y, x)[chan];
}

/* Function that does nothing but demonstrates looping through image coordinates.*/
void loopImage(cv::Mat3b& image)
{
	for (int r = 0; r < image.rows; r++) { // y-coordinate
		for (int c = 0; c < image.cols; c++) { // x-coordinate
			for (int channel = 0; channel < 3; channel++) {
				// DO SOMETHING... example usage
				unsigned char blue = getPixelValue(image, c, r, 0);
				// unsigned char green = getPixelValue(image, c, r, 1); 
				unsigned char red = getPixelValue(image, c, r, 2); 
				image.at<cv::Vec3b>(r, c)[0] = red;
				image.at<cv::Vec3b>(r, c)[2] = blue;
			}
		}
	}
}


// Load Texture by filename and return texture id
unsigned int loadTexture(char* filename)
{
	unsigned int id;
	glGenTextures(1, &id);
	readImage(filename, imageBGR, false);
	// loopImage(imageBGR);
	glBindTexture(GL_TEXTURE_2D, id);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_CLAMP_TO_EDGE);
	glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_CLAMP_TO_EDGE);
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB,
				imageBGR.cols, imageBGR.rows,
				0, GL_RGB, GL_UNSIGNED_BYTE,
				imageBGR.data);
	return id;
}


void initSky()
{
	skybox[SKY_LEFT] = loadTexture("left.jpg");
	skybox[SKY_RIGHT] = loadTexture("right.jpg");
	skybox[SKY_TOP] = loadTexture("top.jpg");
	skybox[SKY_BOTTOM] = loadTexture("bottom.jpg");
	skybox[SKY_FRONT] = loadTexture("front.jpg");
	skybox[SKY_BACK] = loadTexture("back.jpg");
}

void killSky()
{
	glDeleteTextures(6, &skybox[0]);
}

void drawSky(float size)
{
	// GL_TEXTURE_2D state, return back at end
	bool texOn = glIsEnabled(GL_TEXTURE_2D);

	glDisable(GL_LIGHTING);
	glEnable(GL_TEXTURE_2D);
	glColor3f(1.0, 1.0, 1.0);

	// Draw sky front
	glBindTexture(GL_TEXTURE_2D,skybox[SKY_FRONT]);
	glBegin(GL_QUADS);	
		glTexCoord2f(1,0);
		glVertex3f(size/2,size/2,-size/2);
		glTexCoord2f(0,0);
		glVertex3f(-size/2,size/2,-size/2);
		glTexCoord2f(0,1);
		glVertex3f(-size/2,-size/2,-size/2);
		glTexCoord2f(1,1);
		glVertex3f(size/2,-size/2,-size/2);
	glEnd();

	// Draw sky top
	glBindTexture(GL_TEXTURE_2D,skybox[SKY_TOP]);		
	glBegin(GL_QUADS);			
		glTexCoord2f(1,0);
		glVertex3f(size/2,size/2,size/2);
		glTexCoord2f(0,0);
		glVertex3f(-size/2,size/2,size/2);
		glTexCoord2f(0,1);
		glVertex3f(-size/2,size/2,-size/2);
		glTexCoord2f(1,1);
		glVertex3f(size/2,size/2,-size/2);
	glEnd();

	
	// Draw sky left
	glBindTexture(GL_TEXTURE_2D,skybox[SKY_LEFT]);
	glBegin(GL_QUADS);	
		glTexCoord2f(0,0);
		glVertex3f(-size/2,size/2,size/2);
		glTexCoord2f(1,0);
		glVertex3f(-size/2,size/2,-size/2);
		glTexCoord2f(1,1);
		glVertex3f(-size/2,-size/2,-size/2);
		glTexCoord2f(0,1);
		glVertex3f(-size/2,-size/2,size/2);
	glEnd();

	
	// Draw sky right
	glBindTexture(GL_TEXTURE_2D,skybox[SKY_RIGHT]);	
	glBegin(GL_QUADS);	
		glTexCoord2f(0,0);
		glVertex3f(size/2,size/2,-size/2);
		glTexCoord2f(1,0);
		glVertex3f(size/2,size/2,size/2);
		glTexCoord2f(1,1);
		glVertex3f(size/2,-size/2,size/2);
		glTexCoord2f(0,1);
		glVertex3f(size/2,-size/2,-size/2);
	glEnd();

	// Draw sky bottom
	glBindTexture(GL_TEXTURE_2D,skybox[SKY_BOTTOM]);		
	glBegin(GL_QUADS);	
		glTexCoord2f(1,1);
		glVertex3f(size/2,-size/2,size/2);
		glTexCoord2f(0,1);
		glVertex3f(-size/2,-size/2,size/2);
		glTexCoord2f(0,0);
		glVertex3f(-size/2,-size/2,-size/2);
		glTexCoord2f(1,0);
		glVertex3f(size/2,-size/2,-size/2);
	glEnd();

	// Draw sky back 
	glBindTexture(GL_TEXTURE_2D, skybox[SKY_BACK]);
	glBegin(GL_QUADS);
		glTexCoord2f(0,0);
		glVertex3f(size/2,size/2,size/2);	
		glTexCoord2f(1,0);	
		glVertex3f(-size/2,size/2,size/2);
		glTexCoord2f(1,1);
		glVertex3f(-size/2,-size/2,size/2);
		glTexCoord2f(0,1);
		glVertex3f(size/2,-size/2,size/2);
	glEnd();

	if (!texOn)
		glDisable(GL_TEXTURE_2D);
}

int loadSplines(char *argv) {
	char *cName = (char *)malloc(128 * sizeof(char));
	FILE *fileList;
	FILE *fileSpline;
	int iType, i = 0, j, iLength;

	/* load the track file */
	fileList = fopen(argv, "r");
	if (fileList == NULL) {
		printf ("can't open file\n");
		exit(1);
	}
  
	/* stores the number of splines in a global variable */
	fscanf(fileList, "%d", &g_iNumOfSplines);
	printf("%d\n", g_iNumOfSplines);
	g_Splines = (struct spline *)malloc(g_iNumOfSplines * sizeof(struct spline));

	/* reads through the spline files */
	for (j = 0; j < g_iNumOfSplines; j++) {
		i = 0;
		fscanf(fileList, "%s", cName);
		fileSpline = fopen(cName, "r");

		if (fileSpline == NULL) {
			printf ("can't open file\n");
			exit(1);
		}

		/* gets length for spline file */
		fscanf(fileSpline, "%d %d", &iLength, &iType);

		/* allocate memory for all the points */
		g_Splines[j].points = (struct point *)malloc(iLength * sizeof(struct point));
		g_Splines[j].numControlPoints = iLength;

		/* saves the data to the struct */
		while (fscanf(fileSpline, "%lf %lf %lf", 
			&g_Splines[j].points[i].x, 
			&g_Splines[j].points[i].y, 
			&g_Splines[j].points[i].z) != EOF) {
			i++;
		}
	}

	free(cName);

	return 0;
}

/* Write a screenshot to the specified filename */
void saveScreenshot(char *filename)
{
	if (filename == NULL)
		return;

	// Allocate a picture buffer // 
	cv::Mat3b bufferRGB = cv::Mat::zeros(480, 640, CV_8UC3); //rows, cols, 3-channel 8-bit.
	printf("File to save to: %s\n", filename);

	//use fast 4-byte alignment (default anyway) if possible
	glPixelStorei(GL_PACK_ALIGNMENT, (bufferRGB.step & 3) ? 1 : 4);
	//set length of one complete row in destination data (doesn't need to equal img.cols)
	glPixelStorei(GL_PACK_ROW_LENGTH, bufferRGB.step / bufferRGB.elemSize());
	glReadPixels(0, 0, bufferRGB.cols, bufferRGB.rows, GL_RGB, GL_UNSIGNED_BYTE, bufferRGB.data);
	//flip to account for GL 0,0 at lower left
	cv::flip(bufferRGB, bufferRGB, 0);
	//convert RGB to BGR
	cv::Mat3b bufferBGR(bufferRGB.rows, bufferRGB.cols, CV_8UC3);
	cv::Mat3b out[] = { bufferBGR };
	// rgb[0] -> bgr[2], rgba[1] -> bgr[1], rgb[2] -> bgr[0]
	int from_to[] = { 0,2, 1,1, 2,0 };
	mixChannels(&bufferRGB, 1, out, 1, from_to, 3);

	if (cv::imwrite(filename, bufferBGR)) {
		printf("File saved Successfully\n");
	}
	else {
		printf("Error in Saving\n");
	}
}

/* Function to get a pixel value. Use like PIC_PIXEL macro. 
Note: OpenCV images are in channel order BGR. 
This means that:
chan = 0 returns BLUE, 
chan = 1 returns GREEN, 
chan = 2 returns RED. */



void mousedrag(int x, int y)
{

}

void mouseidle(int x, int y)
{
	g_vMousePos[0] = x;
  	g_vMousePos[1] = y;
}


void keyboard(unsigned char key, int x, int y) 
{
    switch (key)
    {
		// case ('w') :
        //     camera.MoveByKey(0);
		// 	break;
        // case ('a') :
        //     camera.MoveByKey(1);
		// 	break;
        // case ('s') :
        //     camera.MoveByKey(2);
		// 	break;
        // case ('d') :
        //     camera.MoveByKey(3);
		// 	break;
		// case ('p') :
		// 	camera.SetLock();
		// 	break;
		default:
			break;
    }
}

void mousebutton(int button, int state, int x, int y)
{

  switch (button)
  {
    case GLUT_LEFT_BUTTON:
      g_iLeftMouseButton = (state==GLUT_DOWN);
      break;
    case GLUT_MIDDLE_BUTTON:
      g_iMiddleMouseButton = (state==GLUT_DOWN);
      break;
    case GLUT_RIGHT_BUTTON:
      g_iRightMouseButton = (state==GLUT_DOWN);
      break;
  }
 
  switch(glutGetModifiers())
  {
    default:
      break;
  }

  g_vMousePos[0] = x;
  g_vMousePos[1] = y;
}


void myinit()
{
  /* setup gl view here */
  	glClearColor(1.0, 1.0, 0.95, 0.0);
    
	initSky();
	costerTex = loadTexture("track.jpg");
  	glEnable(GL_DEPTH_TEST); 
	glShadeModel(GL_SMOOTH);

	bruteForceSplines();  
}

void drawCrossBar() {
	float widthScale = 0.4;
	float sizeScale = .05;
	widthScale /= sizeScale;
	// rail offset
	point offset(0, 0, 0);

	for (int i = 0; i < pList.size() - 1; i += 7)
	{
		point v0, v1, v2, v3, v4, v5, v6, v7;
		point center = pList[i];
		point tan = tanList[i] * sizeScale;
		point bi = biList[i] * sizeScale;
		point nor = norList[i] * sizeScale;
		point left = center + nor * widthScale;
		point right = center - nor * widthScale;

		v0 = left + tan + bi + offset;
		v1 = left - tan + bi + offset;
		v2 = left - tan - bi + offset;
		v3 = left + tan - bi + offset; 

		v4 = right + tan + bi + offset;
		v5 = right - tan + bi + offset;
		v6 = right - tan - bi + offset;
		v7 = right + tan - bi + offset;

		glBegin(GL_QUADS);
			glColor3f(0.6f, 0.6f, 0.6f);
			// Left face
			glVertex3f(v0.x, v0.y, v0.z);
			glVertex3f(v4.x, v4.y, v4.z);
			glVertex3f(v5.x, v5.y, v5.z);
			glVertex3f(v1.x, v1.y, v1.z);
			// Top face
			glVertex3f(v0.x, v0.y, v0.z);
			glVertex3f(v3.x, v3.y, v3.z);
			glVertex3f(v7.x, v7.y, v7.z);
			glVertex3f(v4.x, v4.y, v4.z);
			// Bottom face
			glVertex3f(v1.x, v1.y, v1.z);
			glVertex3f(v2.x, v2.y, v2.z);
			glVertex3f(v6.x, v6.y, v6.z);
			glVertex3f(v5.x, v5.y, v5.z);
			// Right face
			glVertex3f(v2.x, v2.y, v2.z);
			glVertex3f(v3.x, v3.y, v3.z);
			glVertex3f(v7.x, v7.y, v7.z);
			glVertex3f(v6.x, v6.y, v6.z);

		glEnd();
	}
}


void drawLine(int f)
{
	// glBegin(GL_LINE_STRIP);
	// glLineWidth(1.0f);

	float widthScale = 0.4;
	float sizeScale = .05;
	widthScale /= sizeScale;
	// rail offset
	point offset(0, 0, 0);
	
	for (int i = 0; i < pList.size() - 1; i ++)
	{
		point v0, v1, v2, v3, v4, v5, v6, v7;
		point cur = pList[i];
		point next = pList[i + 1];
		point tan = tanList[i] * sizeScale;
		point bi = biList[i] * sizeScale;
		point nor = norList[i] * sizeScale;

		point tanNext = tanList[i + 1] * sizeScale;
		point biNext = biList[i + 1] * sizeScale;
		point norNext = norList[i + 1] * sizeScale;
		
		point nCenterCur = cur + nor * widthScale * f ;
		point nCenterNext = next + norNext * widthScale * f;
		
		if (abs(next.x - cur.x) < 1 && abs(next.y - cur.y) < 1 && abs(next.z - cur.z) < 1) {
			
			v0 = nCenterCur + nor - bi + offset;
			v1 = nCenterCur - nor - bi + offset;
			v2 = nCenterCur - nor + bi + offset;
			v3 = nCenterCur + nor + bi + offset; 

			v4 = nCenterNext + nor - bi + offset;
			v5 = nCenterNext - nor - bi + offset;
			v6 = nCenterNext - nor + bi + offset;
			v7 = nCenterNext + nor + bi + offset;

			glBegin(GL_QUADS);
			glColor3f(0.3f, 0.3f, 0.3f);
			// Left face
			glVertex3f(v0.x, v0.y, v0.z);
			glVertex3f(v4.x, v4.y, v4.z);
			glVertex3f(v5.x, v5.y, v5.z);
			glVertex3f(v1.x, v1.y, v1.z);
			// Top face
			glVertex3f(v0.x, v0.y, v0.z);
			glVertex3f(v3.x, v3.y, v3.z);
			glVertex3f(v7.x, v7.y, v7.z);
			glVertex3f(v4.x, v4.y, v4.z);
			// Bottom face
			glVertex3f(v1.x, v1.y, v1.z);
			glVertex3f(v2.x, v2.y, v2.z);
			glVertex3f(v6.x, v6.y, v6.z);
			glVertex3f(v5.x, v5.y, v5.z);
			// Right face
			glVertex3f(v2.x, v2.y, v2.z);
			glVertex3f(v3.x, v3.y, v3.z);
			glVertex3f(v7.x, v7.y, v7.z);
			glVertex3f(v6.x, v6.y, v6.z);

			glEnd();

		} else 
		{
			printf("large gap, new line\n");
			glEnd();
			glBegin(GL_QUADS);
		}
	}
	glEnd();
}

void moveCam()
{
	point offset(0, 0, 0);
	g_camPos = pList[camIndex] + offset;
	camIndex ++;
	point next;
	next = pList[camIndex] + offset;
	g_camForward = next - g_camPos;
	g_camForward.normalize();

	point bi = biList[camIndex];
	point nor = norList[camIndex];

	g_camPos = g_camPos + nor * 0.5;

	g_camUp.crossProduct(g_camForward, nor);
	g_camUp.normalize();

	g_camRight.crossProduct(g_camUp, g_camForward);
	g_camRight.normalize();

	if (camIndex >= pList.size() - 1) {
		camIndex = 0;
	}

	gluLookAt(g_camPos.x, g_camPos.y, g_camPos.z,
			  g_camPos.x + g_camForward.x, g_camPos.y + g_camForward.y, g_camPos.z + g_camForward.z,
			  g_camUp.x, g_camUp.y, g_camUp.z);
}


void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();

    gluLookAt(0.5f, 1.0f, 1.0f, 0.5f, 0.5f, 0.5f, 0.0f, 1.0f, 0.0f);

    glTranslatef(g_vLandTranslate[0], g_vLandTranslate[1], g_vLandTranslate[2]);
    glRotatef(g_vLandRotate[0], 1, 0, 0);
    glRotatef(g_vLandRotate[1], 0, 1, 0);
    glRotatef(g_vLandRotate[2], 0, 0, 1);
    glScalef(g_vLandScale[0], g_vLandScale[1], g_vLandScale[2]);
	
	
	// Update Camera
	// camera.Control(g_vMousePos[0], g_vMousePos[1]);
	// camera.UpdateCamera();
	moveCam();

	// Render track
	drawLine(1);
	drawLine(-1);
	drawCrossBar();
	
	// Render Sky
	drawSky(100);
    
	glutSwapBuffers(); 
}

void reshape(int w, int h)
{
	glViewport(0, 0, w, h);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();

	gluPerspective(60.0f, w / h, 0.1f, 100.0f);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
}

void bruteForceSplines()
{
	for (int i = 0; i < g_iNumOfSplines; i++)
	{
		for (int j = 0; j < g_Splines[i].numControlPoints - 3; j++)
		{
			point a = g_Splines[i].points[j];
			point b = g_Splines[i].points[j + 1];
			point c = g_Splines[i].points[j + 2];
			point d = g_Splines[i].points[j + 3];

			point tan, nor, bi;
			bi = point(0, 1, 0);

			for (double u = 0; u < 1.0; u += 0.04)
			{
				point x;
				x.generatePoints(u, a, b, c, d);
				pList.push_back(x);
				
				tan.tangent(u, a, b, c, d);
				tanList.push_back(tan);

				nor.normal(tan, bi, 1);
				norList.push_back(nor);

				bi.binormal(tan, nor);
				biList.push_back(bi);
			}
		}
	}
}



/* OpenCV help:
Access number of rows of image (height): image.rows; 
Access number of columns of image (width): image.cols;
Pixel 0,0 is the upper left corner. Byte order for 3-channel images is BGR. 
*/

void menufunc(int value)
{
  switch (value)
  {
    case 0:
      exit(0);
      break;
	default:
		break;
  }
}


void doIdle()
{
  /* do some stuff... */
    // camera.MouseIn(true);
  /* make the screen update */
  glutPostRedisplay();
}



int _tmain(int argc, _TCHAR* argv[])
{
	// I've set the argv[1] to track.txt.
	// To change it, on the "Solution Explorer",
	// right click "assign1", choose "Properties",
	// go to "Configuration Properties", click "Debugging",
	// then type your track file name for the "Command Arguments"
	if (argc<2)
	{  
		printf ("usage: %s <trackfile>\n", argv[0]);
		exit(0);
	}
	
	loadSplines(argv[1]);

	glutInit(&argc,(char**)argv);
  
	/*
		create a window here..should be double buffered and use depth testing
  
	    the code past here will segfault if you don't have a window set up....
	    replace the exit once you add those calls.
	*/
  	glutInitDisplayMode(GLUT_DOUBLE | GLUT_DEPTH | GLUT_RGBA);
	glutInitWindowSize(640, 480);
	glutInitWindowPosition(0, 0);
	glutCreateWindow("Assignment 2");

	/* tells glut to use a particular display function to redraw */
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);

	// If you need to load textures use below instead of pic library:
	// readImage("spiral.jpg", imageBGR, true);

	// Demonstrates to loop across image and access pixel values:
	// Note this function doesn't do anything, but you may find it useful:
	// loopImage(imageBGR);

	// Rebuilt save screenshot function, but will seg-fault unless you have
	// OpenGL framebuffers initialized. You can use this new function as before:
	// saveScreenshot("test_screenshot.jpg");

	/* replace with any animate code */
	glutIdleFunc(doIdle);

	/* callback for mouse drags */
	glutMotionFunc(mousedrag);
	/* callback for idle mouse movement */
	glutPassiveMotionFunc(mouseidle);
	/* callback for mouse button changes */
	glutMouseFunc(mousebutton);
    glutKeyboardFunc(keyboard);

	/* do initialization */
	myinit();

	glutMainLoop();
	return 0;
}
