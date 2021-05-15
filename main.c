/**
 * Experiment using signed distance functions to draw... very slowly...
 * Author: JPK - james@jamespking.com
 *
 * Licence: This work is licensed under the Creative Commons Attribution License.
 *           View this license at http://creativecommons.org/about/licenses/
 *
 *
 *
 * Acknowledgements:
 *
 * Inspired by the brilliant computer graphics artist Inigo Quilez (https://www.iquilezles.org/). 
 *
 * Ray marching algo adapted from http://jamie-wong.com/2016/07/15/ray-marching-signed-distance-functions/
 *
 * Primitive SDFs from https://iquilezles.org/www/articles/distfunctions/distfunctions.htm
 *
 * LCD code by Steve Gunn
 *
 * Build/Makefile by Klaus-Peter Zauner
 *
 * # Motivation
 * I found Inigo Quilez's Youtube channel earlier on in the year thanks to Tristan Hume's blog (https://thume.ca). Since then, I've been
 * interested in taking a look at the techniques he uses and SDFs are a major part of that. I've kept this very basic, it just renders
 * SDFs using a colour which is dependent distance a ray traveled (no lighting effects, until you add them). It's slow, but it was fun
 * to implement.
 *
 * # What you'll need
 * Your La Fortuna (board with a AT90USB1286 microcontroller) and the toolchain configured.
 *
 * Fork the repo, clone, and then just make.
 *
 * # Signed Distance Functions
 * The name gives it away. These functions return the shortest distance to the surface of a shape from a given point in space.
 * When the point is inside the shape, they are negative. We can combine SDFs for different shapes to make a more complex scene SDF.
 *
 * # Drawing with SDFs
 * To get an image we need to render the SDF. To do this we consider the grid of pixels which make the display and put a camera in front.
 * The camera iteratively considers rays going through these pixels, and the ray marching algorithm is used to decide if the ray collides
 * with an object and if so what the depth is.
 *
 * # Ray marching algorithm
 * The goal of ray marching is to decide if a ray collides with an object in the scene and if so what the depth is.
 * Each ray is described by a direction vector. Since the SDF returns the shortest distance to the surface of an object, we can safely move
 * that far along the direction vector. This happens iteratively until either the SDF returns (about) 0 (indicating that we are at the surface),
 * the number of iterations reaches a limit, or the total distance moved exceeds the draw distance.
 *
 */

#include <avr/io.h>
#include "lcd/lcd.h"
#include "math.h"


const int camera_x = 159;
const int camera_y = 119;
const int camera_z = -100;
const int ep = 2;
const int MAX_MARCH = 50;
const double MAX_DEPTH = 180;

void init(void);
double raymarch(int x, int y, int t);
double sdf(double x, double y, double z, int t);
double unionSDF(double a, double b);
double differenceSDF(double a, double b);
double sphereSDF(double p_x, double p_y, double p_z, double radius);
double torusSDF(double x, double y, double z, double r, double t);


void main(void) {
	uint16_t color;
	init();

	// Add a background
	rectangle bg = {0,319,0,239};
	fill_rectangle(bg, BLACK);


	// time ticks
	for (int t = 0; t <= 1000; t += 1) {
		// loop through x and y axis, uses bigger step size to speed up rendering
		for (int x = 0; x < 320; x += 10) {
			for (int y = 0; y < 240; y += 10) {
				double distance = raymarch(x, y, t);
				// 'pixel' being drawn, bigger so it looks better with the big step size
				rectangle pix = {x, x+2,y,y+2};

				if (distance < MAX_DEPTH) {
					// jank colour choice, based on distance. From white to black depending on distance.
					if (distance < 0) {
						color = WHITE;
					} else if (distance < 100) {
						color = ((31 - (uint16_t) fabs(distance * 0.31)) << 11) // red
							+ ((63 - (uint16_t) fabs(distance * 0.63)) << 5) // green 
							+ (31 - (uint16_t) fabs(distance * 0.2)); // blue
					} else if (distance < 300) {
						color = BLACK;
					}

					fill_rectangle(pix, color);
				} else {
					// no collision, colour changes so you can see backdrop being drawn 
					fill_rectangle(pix, 0x0005 + 5*t);
				}

			} 
		}
	}
}

// Decide if a ray, going through pixel (x, y) at time t collides with an object in the scene and if so what the depth is
double raymarch(int x, int y, int t) {
	// Normalise the vector pointing from the camera to the pixel at (x, y, 0), so we have a direction
	double dir_magnitude = sqrt((x - camera_x)*(x - camera_x) + (y - camera_y)*(y - camera_y) + camera_z*camera_z);
	double dir_x = (x - camera_x) / dir_magnitude;
	double dir_y = (y - camera_y) / dir_magnitude;
	double dir_z = (0 - camera_z) / dir_magnitude;

	double depth = 0;

	// Step along the ray
	for (int i = 0; i < MAX_MARCH; i++) {
		// Distance to nearest scene surface from the point which is distance depth in previously calculated direction
		double distanceSDF = sdf(camera_x + depth * dir_x, camera_y + depth * dir_y, camera_z + depth * dir_z, t); 

		// If distance is small we are done
		if (distanceSDF < ep) {
			return depth;
		}

		depth += distanceSDF;

		if (depth >= MAX_DEPTH) {
			return MAX_DEPTH;
		}
	}

	return MAX_DEPTH;
}


// Returns the distance of the closest surface to the point (x, y, z) at time t
// This is where we set up the sample scene (rotating donuts)
double sdf(double x, double y, double z, int t) {
	// Set the center for each object we put in the scene
	// In this case, I set all the objects to be centered in the screen
	double p_x = x - camera_x;
	double p_y = y - camera_y;
	double p_z = z - camera_z - 60; // and 60px behind the screen
	double theta = 10 * t * 0.017; // angle for rotations, moves in steps of 10 deg, translated to radians
	double bob = sin(t * 0.017 * 10) * 15; // for periodic motion
	double c = cos(theta); // for rotations
	double s = sin(theta);

	// The scene is build from the union of two torus primitives and a sphere
	// Each torus is rotated, the first about the x axis and the other about y
	// 	These are calculated by multiplying out the elemental rotation matrices with 
	// 	the origin of the object
	// The sphere bobs up and down by changing its y position
	return unionSDF(torusSDF(p_x, c * p_y - s * p_z, s * p_y + c * p_z, 30, 5),
			unionSDF(sphereSDF(p_x, p_y + bob, p_z, 10),
				torusSDF(c * p_x + s * p_z, p_y, -s * p_x + c * p_z, 45, 5)));
}


// Primitive SDFs and combinations
double unionSDF(double a, double b) {
	return fmin(a, b);
}

double differenceSDF(double a, double b) {
	return fmax(a, -b);
}

double sphereSDF(double p_x, double p_y, double p_z, double radius) {
	return sqrt(p_x*p_x + p_y*p_y + p_z*p_z) - radius;
}

// Donut shape.
// y, z switched since we look from point on x axis, so rotates to stand up
double torusSDF(double x, double z, double y, double r, double t) {
	double q_x = sqrt(x*x + z*z) - r;
	double q_y = y;
	return sqrt(q_x*q_x + q_y*q_y) - t;
}


void init(void) {    
	/* 8MHz clock, no prescaling (DS, p. 48) */
	/* How do I overclock this thing? ;) */
	CLKPR = (1 << CLKPCE);
	CLKPR = 0;

	init_lcd();
} 
