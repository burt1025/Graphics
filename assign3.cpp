/*
CSCI 480
Assignment 3 Raytracer

Name: Buting XU
*/

#include <windows.h>
#include <stdlib.h>
#include <GL/glu.h>
#include <GL/glut.h>

#include <stdio.h>
#include <string>

#include "opencv2/core/core.hpp"
#include "opencv2/core/utility.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgcodecs/imgcodecs.hpp"

#define MAX_TRIANGLES 2000
#define MAX_SPHERES 10
#define MAX_LIGHTS 10
#define NUM_SAMPLE 40

bool g_anti = true;
bool g_soft = true;

char *filename=0;

//different display modes
#define MODE_DISPLAY 1
#define MODE_JPEG 2
int mode=MODE_DISPLAY;

//you may want to make these smaller for debugging purposes
#define WIDTH 640
#define HEIGHT 480
#define ASPECT_RATIO 640.0/480.0
#define fov 60.0

#define PI 3.1415926535
#define ALPHA std::tan((fov/2.0) * (PI/180.0))

#define g_DEPTH 4

unsigned char buffer[HEIGHT][WIDTH][3];
unsigned int g_x,g_y;

double cap(double a) {
  if (a > 1.0) 
    return 1.0;
  else if (a < 0.0) 
    return 0.0;
  else 
    return a;
}

struct Vertex
{
  double position[3];
  double color_diffuse[3];
  double color_specular[3];
  double normal[3];
  double shininess;
};

struct Vector3
{
  double x;
  double y;
  double z;

  Vector3() : x(0), y(0), z(0) {}

  Vector3(double a, double b, double c) : x(a), y(b), z(c) {}

  Vector3 operator+(const Vector3 b) const{
    return Vector3(x + b.x, y + b.y, z + b.z);
  }

  Vector3 operator-(const Vector3 b) const {
    return Vector3(x - b.x, y - b.y, z - b.z);
  }

  Vector3 operator*(float f) const {
    return Vector3(x * f, y * f, z * f);
  }

  // Matrix multiply
  Vector3 operator*(Vector3 b) const {
    return Vector3(x * b.x, y * b.y, z * b.z);
  }

  Vector3 operator-() const {
    return Vector3(-x, -y, -z);
  } 

  Vector3& operator=(const Vector3& b){
    x = b.x;
    y = b.y;
    z = b.z;
    return *this;
  }

  Vector3& operator+=(const Vector3& b) {
    x += b.x;
    y += b.y;
    z += b.z;
    return *this;
  }



  bool operator==(const Vector3 b) const {
    return (x == b.x && y == b.y && z == b.z);
  }

  bool haszero()
  {
    return (x < 0 || y < 0 || z < 0);
  }

  double getLength(){
    return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
  }

  double getSum(){
    return x + y + z;
  }

  void print() {
    printf("%f, %f, %f\n", x, y, z);
  }

  Vector3 normalize(){
    double length = getLength();
    return Vector3(x / length, y / length, z / length);
  }

  void selfcap() {
    x = cap(x);
    y = cap(y);
    z = cap(z);
  }
};

Vector3 g_backGround(1.0, 1.0, 0.95);

double DotProduct(Vector3 a, Vector3 b){
  return a.x * b.x + a.y * b.y + a.z * b.z;
}

Vector3 CrossProduct(Vector3 a, Vector3 b) {
  return Vector3(a.y * b.z - a.z * b.y, a.z * b.x -a.x * b.z, a.x * b.y - a.y * b.x);
}

struct Color
{
  double r;
  double g;
  double b;

  Color() {}

  Color(double r1, double g1, double b1) : r(r1), b(b1), g(g1) {}

  Color(Vector3 v) : r(v.x), g(v.y), b(v.z) {}

  Color& operator+= (const Color& a) {
    r += a.r;
    g += a.g;
    b += a.g;
    r = cap(r);
    g = cap(g);
    b = cap(b);
    return *this;
  }
};

typedef struct _Triangle
{
  struct Vertex v[3];
} Triangle;

typedef struct _Sphere
{
  double position[3];
  double color_diffuse[3];
  double color_specular[3];
  double shininess;
  double radius;
} Sphere;

typedef struct _Light
{
  double position[3];
  double color[3];
} Light;

struct Material
{
  Vector3 diffuse;
  Vector3 specular;
  double shininess;

  Material() {};

  Material(Vector3 d, Vector3 spec, double s) : diffuse(d), specular(spec), shininess(s) {};

  Material(_Sphere s) {
    Vector3 diff(s.color_diffuse[0], s.color_diffuse[1], s.color_diffuse[2]);
    Vector3 spec(s.color_specular[0], s.color_specular[1], s.color_specular[2]);
    diffuse = diff;
    specular = spec;
    shininess = s.shininess;
  }

  Material(_Triangle triangle) {
    Vector3 diff(triangle.v[0].color_diffuse[0], triangle.v[0].color_diffuse[1], triangle.v[0].color_diffuse[2]);
    Vector3 spec(triangle.v[0].color_specular[0], triangle.v[0].color_specular[1],triangle.v[0].color_specular[2]);
    diffuse = diff;
    specular = spec;
    shininess = triangle.v[0].shininess;
  }
};

Triangle triangles[MAX_TRIANGLES];
Sphere spheres[MAX_SPHERES];
Light lights[MAX_LIGHTS];
double ambient_light[3];

int num_triangles=0;
int num_spheres=0;
int num_lights=0;

Vector3 reflect(const Vector3 &I, const Vector3 &N)
{
  return I - N * 2.0 * DotProduct(I, N);
}

struct Ray{
  Vector3 ori;
  Vector3 dir;

  Ray() {}

  Ray(Vector3 nOri, Vector3 nDir) : ori(nOri), dir(nDir) {}

  bool Intersect(_Sphere sphere, double &t0) {
    Vector3 l = Vector3(sphere.position[0], sphere.position[1], sphere.position[2]) - ori;
    double tca = DotProduct(l, dir);
    double d2 = DotProduct(l, l) - tca * tca;
    if (d2 > sphere.radius * sphere.radius) return false;

    double thc = sqrt(sphere.radius * sphere.radius - d2);
    t0 = tca - thc;
    float t1 = tca + thc;
    if (t0 < 0) t0 = t1;
    if (t0 < 0) return false;
    return true;
  }

  bool Intersect(_Triangle triangle, double &t) {
    Vector3 v0 = Vector3(triangle.v[0].position[0], triangle.v[0].position[1], triangle.v[0].position[2]);
    Vector3 v1 = Vector3(triangle.v[1].position[0], triangle.v[1].position[1], triangle.v[1].position[2]);
    Vector3 v2 = Vector3(triangle.v[2].position[0], triangle.v[2].position[1], triangle.v[2].position[2]);
  
    Vector3 normal = CrossProduct(v1 - v0, v2 - v0);

    if( std::abs(DotProduct(normal, dir)) < 1e-6) {
      return false;
    } 

    t = - (DotProduct((ori - v0), normal.normalize()) / DotProduct(normal.normalize(), dir));

    if (t < 0.001) return false;

    Vector3 hit = ori + dir * t;
    Vector3 C;

    C = CrossProduct(v1 - v0, hit - v0);
    if (DotProduct(normal, C) < 0) return false;

    C = CrossProduct(v2 - v1, hit - v1);
    if (DotProduct(normal, C) < 0) return false;

    C = CrossProduct(v0 - v2, hit - v2);
    if (DotProduct(normal, C) < 0) return false;

    return true;
  }
};

Ray genRay(double x, double y, double xOffset = 0.5, double yOffset = 0.5)
{
  double newX = (2 * ((x + xOffset) / (double) WIDTH) - 1) * ASPECT_RATIO * ALPHA;
  double newY = (2 * ((y + yOffset) / (double) HEIGHT) - 1) * ALPHA;
  Vector3 ori(0.0, 0.0, 0.0);
  Vector3 dir = Vector3(newX, newY, -1.0).normalize();

  return Ray(ori, dir);
}

std::vector<Light> sample(Light light) {
  std::vector<Light> samples;

  Vector3 light_pos(light.position[0], light.position[1], light.position[2]);
  Vector3 light_color(light.color[0], light.color[1], light.color[2]);
  for (int i=0; i < NUM_SAMPLE; i++) {
  	double radius = ((double) rand() / (RAND_MAX)) / 10.0;
  	double theta = ((double) (rand() % 360) * PI/180.0 );
  	double phi = ((double) (rand() % 360) * PI/180.0);

    Vector3 offset( radius * std::sin(theta) * std::cos(phi), 
          radius * std::sin(theta) * std::sin(phi),
          radius * std::cos(theta));

    Vector3 newPos = light_pos + offset;

    Vector3 newCol = light_color * (1.0 / NUM_SAMPLE);

  	Light newLight;
  	newLight.position[0] = newPos.x;
  	newLight.position[1] = newPos.y;
  	newLight.position[2] = newPos.z;
  	newLight.color[0] = newCol.x;
  	newLight.color[1] = newCol.y;
  	newLight.color[2] = newCol.z;
  	samples.push_back(newLight);
  }
  return samples;
}

void getIntersect(_Triangle triangle, Vector3 intersect, Vector3& normal, Material&mat)
{
  Vector3 v0(triangle.v[0].position[0], triangle.v[0].position[1], triangle.v[0].position[2]);
  Vector3 v1(triangle.v[1].position[0], triangle.v[1].position[1], triangle.v[1].position[2]);
  Vector3 v2(triangle.v[2].position[0], triangle.v[2].position[1], triangle.v[2].position[2]);

  Vector3 pNormal = CrossProduct( (v1 - v0), (v2 - v0));
  double pNorSqr = DotProduct(pNormal, pNormal);

  double alpha = DotProduct(pNormal, CrossProduct(v2 - v1, intersect - v1)) / pNorSqr;
  double beta = DotProduct(pNormal, CrossProduct(v0 - v2, intersect - v2)) / pNorSqr;
  double gamma = 1.0 - alpha - beta;

  Vector3 sectNormal(alpha * triangle.v[0].normal[0] + beta * triangle.v[1].normal[0] + gamma * triangle.v[2].normal[0],
  					  alpha * triangle.v[0].normal[1] + beta * triangle.v[1].normal[1] + gamma * triangle.v[2].normal[1],
  					  alpha * triangle.v[0].normal[2] + beta * triangle.v[1].normal[2] + gamma * triangle.v[2].normal[2]);

  normal = sectNormal.normalize();


  Vector3 diffuse(alpha * triangle.v[0].color_diffuse[0] + beta * triangle.v[1].color_diffuse[0] + gamma * triangle.v[2].color_diffuse[0],
  				alpha * triangle.v[0].color_diffuse[1] + beta * triangle.v[1].color_diffuse[1] + gamma * triangle.v[2].color_diffuse[1],
  				alpha * triangle.v[0].color_diffuse[2] + beta * triangle.v[1].color_diffuse[2] + gamma * triangle.v[2].color_diffuse[2]);

  Vector3 specular(alpha * triangle.v[0].color_specular[0] + beta * triangle.v[1].color_specular[0] + gamma * triangle.v[2].color_specular[0],
  				 alpha * triangle.v[0].color_specular[1] + beta * triangle.v[1].color_specular[1] + gamma * triangle.v[2].color_specular[1],
  				 alpha * triangle.v[0].color_specular[2] + beta * triangle.v[1].color_specular[2] + gamma * triangle.v[2].color_specular[2]);

  double s = alpha * triangle.v[0].shininess + 
                      beta * triangle.v[1].shininess + 
                      gamma * triangle.v[2].shininess;

  mat = Material(diffuse, specular, s);
}

bool sceneIntersect(Ray ray, Vector3& hit, Vector3& normal, Material& mat, bool& istriangle) {
  double shortest_dist = std::numeric_limits<double>::max();
  istriangle = false;
  
  for (int i = 0; i < num_spheres; i ++) {
    double dist;
    if (ray.Intersect(spheres[i], dist) && dist < shortest_dist) {
      shortest_dist = dist;
      hit = ray.ori + ray.dir * dist;
      normal = (hit - Vector3(spheres[i].position[0], spheres[i].position[1], spheres[i].position[2])).normalize();
      mat = Material(spheres[i]);
    }
  }

  for (int j = 0; j < num_triangles; j ++) {
    double dist;
    if (ray.Intersect(triangles[j], dist) && dist < shortest_dist) {
      shortest_dist = dist;
      hit = ray.ori + ray.dir * dist;
      getIntersect(triangles[j], hit, normal, mat);
      istriangle = true;
    }
  }

  return shortest_dist < 1000;
}

Vector3 castRay(Ray ray, int depth = 0) {
  Vector3 point, N;
  Material mat;
  bool dummy;

  if ( !sceneIntersect(ray, point, N, mat, dummy)) {
    return Vector3(g_backGround);
  }
  // Recursive
  // Vector3 reflect_dir = reflect(ray.dir, N);
  // Vector3 refelct_ori = DotProduct(reflect_dir, N) < 0 ? point - N*1e-3 : point + N*1e-3;
  // Color reflect_color = castRay(Ray(refelct_ori, reflect_dir), depth + 1); 

  Vector3 diff_light_color(0, 0, 0);
  Vector3 spec_light_color(0, 0, 0);

  Vector3 g_ambient(ambient_light[0], ambient_light[1], ambient_light[2]);

  for (unsigned int i = 0; i < num_lights; i++) {
    if (g_soft) {
      std::vector<Light> samples = sample(lights[i]); 

      for (int l = 0; l < samples.size(); l ++) {
        Vector3 light_ori = Vector3(samples[l].position[0], samples[l].position[1], samples[l].position[2]);
        Vector3 light_dir = (light_ori - point).normalize();
        Vector3 light_color(samples[l].color[0], samples[l].color[1], samples[l].color[2]);
        double light_distance = (light_ori - point).getLength();

        // // Shadow
        Vector3 shadow_ori = DotProduct(light_dir, N) < 0 ? point - N * 1e-3 : point + N * 1e-3;
        Vector3 shadow_pt, shadow_N;
        Material tempMat;
        bool shadowByTri;
        if (sceneIntersect(Ray(shadow_ori, light_dir), shadow_pt, shadow_N, tempMat, shadowByTri)&& 
          (shadow_pt - shadow_ori).getLength() < light_distance)
        {
          continue;
        }
        // phong lighting
        double intensity = cap(DotProduct(light_dir, N));
        double reflection = cap(DotProduct(reflect(light_dir, N), ray.dir.normalize()));

        diff_light_color += light_color * intensity;
        spec_light_color += light_color * std::powf(reflection, mat.shininess);
      }
    } 
    else {
      Vector3 light_ori = Vector3(lights[i].position[0], lights[i].position[1], lights[i].position[2]);
      Vector3 light_dir = (light_ori - point).normalize();
      Vector3 light_color(lights[i].color[0], lights[i].color[1], lights[i].color[2]);
      double light_distance = (light_ori - point).getLength();

      // // Shadow
      Vector3 shadow_ori = DotProduct(light_dir, N) < 0 ? point - N * 1e-3 : point + N * 1e-3;
      Vector3 shadow_pt, shadow_N;
      Material tempMat;
      bool shadowByTri;
      if (sceneIntersect(Ray(shadow_ori, light_dir), shadow_pt, shadow_N, tempMat, shadowByTri)&& 
        (shadow_pt - shadow_ori).getLength() < light_distance)
      {
        continue;
      }
      double intensity = cap(DotProduct(light_dir, N));
      double reflection = cap(DotProduct(reflect(light_dir, N), ray.dir.normalize()));

      diff_light_color += light_color * intensity;
      spec_light_color += light_color * std::powf(reflection, mat.shininess);
    }
  }

  Vector3 pixColor(0.0, 0.0, 0.0);

  pixColor = diff_light_color * mat.diffuse + (spec_light_color + Vector3(0.1, 0.1, 0.1)) * mat.specular + g_ambient;
  pixColor.selfcap();
  return pixColor;
}



void plot_pixel_display(int x,int y,unsigned char r,unsigned char g,unsigned char b);
void plot_pixel_jpeg(int x,int y,unsigned char r,unsigned char g,unsigned char b);
void plot_pixel(int x,int y,unsigned char r,unsigned char g,unsigned char b);

//MODIFY THIS FUNCTION
void draw_scene()
{
  
  //simple output
  for(g_x=0; g_x<WIDTH; g_x++)
  {
    glPointSize(2.0);  
    glBegin(GL_POINTS);
    for(g_y=0;g_y < HEIGHT;g_y++)
    {
      if (g_anti)
      {
        Vector3 color;
        Ray ray1 = genRay(g_x, g_y, 0.25, 0.25);
        Ray ray2 = genRay(g_x, g_y, 0.75, 0.25);
        Ray ray3 = genRay(g_x, g_y, 0.25, 0.75);
        Ray ray4 = genRay(g_x, g_y, 0.75, 0.75);
        color += castRay(ray1);
        color += castRay(ray2);
        color += castRay(ray3);
        color += castRay(ray4);
        color = color * 0.25;

        plot_pixel(g_x, g_y, color.x * 255, color.y * 255, color.z * 255);
      } 
      else {
        Vector3 color;
        Ray ray = genRay(g_x, g_y);
        color = castRay(ray);
        plot_pixel(g_x, g_y, color.x * 255, color.y * 255, color.z * 255);
      }
    }
    glEnd();
    glFlush();
  }
  printf("Done!\n"); fflush(stdout);
}

void plot_pixel_display(int x,int y,unsigned char r,unsigned char g,unsigned char b)
{
  glColor3f(((double)r)/256.f,((double)g)/256.f,((double)b)/256.f);
  glVertex2i(x,y);
}

void plot_pixel_jpeg(int x,int y,unsigned char r,unsigned char g,unsigned char b)
{
  buffer[HEIGHT-y-1][x][0]=r;
  buffer[HEIGHT-y-1][x][1]=g;
  buffer[HEIGHT-y-1][x][2]=b;
}

void plot_pixel(int x,int y,unsigned char r,unsigned char g, unsigned char b)
{
  plot_pixel_display(x,y,r,g,b);
  if(mode == MODE_JPEG)
      plot_pixel_jpeg(x,y,r,g,b);
}

/* Write a jpg image from buffer*/
void save_jpg()
{
	if (filename == NULL)
		return;

	// Allocate a picture buffer // 
	cv::Mat3b bufferBGR = cv::Mat::zeros(HEIGHT, WIDTH, CV_8UC3); //rows, cols, 3-channel 8-bit.
	printf("File to save to: %s\n", filename);

	// unsigned char buffer[HEIGHT][WIDTH][3];
	for (int r = 0; r < HEIGHT; r++) {
		for (int c = 0; c < WIDTH; c++) {
			for (int chan = 0; chan < 3; chan++) {
				unsigned char red = buffer[r][c][0];
				unsigned char green = buffer[r][c][1];
				unsigned char blue = buffer[r][c][2];
				bufferBGR.at<cv::Vec3b>(r,c) = cv::Vec3b(blue, green, red);
			}
		}
	}
	if (cv::imwrite(filename, bufferBGR)) {
		printf("File saved Successfully\n");
	}
	else {
		printf("Error in Saving\n");
	}
}

void parse_check(char *expected,char *found)
{
  if(stricmp(expected,found))
    {
      char error[100];
      printf("Expected '%s ' found '%s '\n",expected,found);
      printf("Parse error, abnormal abortion\n");
      exit(0);
    }

}

void parse_doubles(FILE*file, char *check, double p[3])
{
  char str[100];
  fscanf(file,"%s",str);
  parse_check(check,str);
  fscanf(file,"%lf %lf %lf",&p[0],&p[1],&p[2]);
  printf("%s %lf %lf %lf\n",check,p[0],p[1],p[2]);
}

void parse_rad(FILE*file,double *r)
{
  char str[100];
  fscanf(file,"%s",str);
  parse_check("rad:",str);
  fscanf(file,"%lf",r);
  printf("rad: %f\n",*r);
}

void parse_shi(FILE*file,double *shi)
{
  char s[100];
  fscanf(file,"%s",s);
  parse_check("shi:",s);
  fscanf(file,"%lf",shi);
  printf("shi: %f\n",*shi);
}

int loadScene(char *argv)
{
  FILE *file = fopen(argv,"r");
  int number_of_objects;
  char type[50];
  int i;
  Triangle t;
  Sphere s;
  Light l;
  fscanf(file,"%i",&number_of_objects);

  printf("number of objects: %i\n",number_of_objects);
  char str[200];

  parse_doubles(file,"amb:",ambient_light);

  for(i=0;i < number_of_objects;i++)
  {
    fscanf(file,"%s\n",type);
    printf("%s\n",type);
    if(stricmp(type,"triangle")==0)
	  {

      printf("found triangle\n");
      int j;

      for(j=0;j < 3;j++)
      {
        parse_doubles(file,"pos:",t.v[j].position);
        parse_doubles(file,"nor:",t.v[j].normal);
        parse_doubles(file,"dif:",t.v[j].color_diffuse);
        parse_doubles(file,"spe:",t.v[j].color_specular);
        parse_shi(file,&t.v[j].shininess);
      }

      if(num_triangles == MAX_TRIANGLES)
      {
        printf("too many triangles, you should increase MAX_TRIANGLES!\n");
        exit(0);
      }
      triangles[num_triangles++] = t;
	  }
    else if(stricmp(type,"sphere")==0)
    {
      printf("found sphere\n");

      parse_doubles(file,"pos:",s.position);
      parse_rad(file,&s.radius);
      parse_doubles(file,"dif:",s.color_diffuse);
      parse_doubles(file,"spe:",s.color_specular);
      parse_shi(file,&s.shininess);

      if(num_spheres == MAX_SPHERES)
      {
        printf("too many spheres, you should increase MAX_SPHERES!\n");
        exit(0);
      }
      spheres[num_spheres++] = s;
    }
    else if(stricmp(type,"light")==0)
    {
      printf("found light\n");
      parse_doubles(file,"pos:",l.position);
      parse_doubles(file,"col:",l.color);

      if(num_lights == MAX_LIGHTS)
      {
        printf("too many lights, you should increase MAX_LIGHTS!\n");
        exit(0);
      }
      lights[num_lights++] = l;
    }
    else
    {
      printf("unknown type in scene description:\n%s\n",type);
      exit(0);
    }
  }
  return 0;
}

void display()
{

}

void init()
{
  glMatrixMode(GL_PROJECTION);
  glOrtho(0,WIDTH,0,HEIGHT,1,-1);
  glMatrixMode(GL_MODELVIEW);
  glLoadIdentity();

  glClearColor(0,0,0,0);
  glClear(GL_COLOR_BUFFER_BIT);
}

void idle()
 {
  //hack to make it only draw once
  static int once=0;
  if(!once)
  {
      draw_scene();
      if(mode == MODE_JPEG)
	      save_jpg();
    }
  once=1;
}

int main (int argc, char ** argv)
{
  if (argc < 2 || argc > 3)
  {  
    printf ("usage: %s <scenefile> [jpegname]\n", argv[0]);
    exit(0);
  }
  if(argc == 3)
    {
      mode = MODE_JPEG;
      filename = argv[2];
    }
  else if(argc == 2)
    mode = MODE_DISPLAY;
  
  glutInit(&argc,argv);
  loadScene(argv[1]);

  glutInitDisplayMode(GLUT_RGBA | GLUT_SINGLE);
  glutInitWindowPosition(0,0);
  glutInitWindowSize(WIDTH,HEIGHT);
  int window = glutCreateWindow("Ray Tracer");
  glutDisplayFunc(display);
  glutIdleFunc(idle);
  init();
  glutMainLoop();
}	
