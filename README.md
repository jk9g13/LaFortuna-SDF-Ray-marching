# LaFortuna-SDF-Ray-marching
Experiment using signed distance functions to draw... very slowly...
  Experiment using signed distance functions to draw... very slowly... 
  Author: JPK - james@jamespking.com 
  
  Licence: This work is licensed under the Creative Commons Attribution License. 
            View this license at http://creativecommons.org/about/licenses/ 
  
  
  
  Acknowledgements: 
  
  Inspired by the brilliant computer graphics artist Inigo Quilez (https://www.iquilezles.org/).  
  
  Ray marching algo adapted from http://jamie-wong.com/2016/07/15/ray-marching-signed-distance-functions/ 
  
  Primitive SDFs from https://iquilezles.org/www/articles/distfunctions/distfunctions.htm 
  
  LCD code by Steve Gunn 
  
  Build/Makefile by Klaus-Peter Zauner 
  
  # Motivation 
  I found Inigo Quilez's Youtube channel earlier on in the year thanks to Tristan Hume's blog (https://thume.ca). Since then, I've been 
  interested in taking a look at the techniques he uses and SDFs are a major part of that. I've kept this very basic, it just renders 
  SDFs using a colour which is dependent distance a ray traveled (no lighting effects, until you add them). It's slow, but it was fun 
  to implement. 
  
  # What you'll need 
  Your La Fortuna (board with a AT90USB1286 microcontroller) and the toolchain configured. 
  
  Fork the repo, clone, and then just make. 
  
  # Signed Distance Functions 
  The name gives it away. These functions return the shortest distance to the surface of a shape from a given point in space. 
  When the point is inside the shape, they are negative. We can combine SDFs for different shapes to make a more complex scene SDF. 
  
  # Drawing with SDFs 
  To get an image we need to render the SDF. To do this we consider the grid of pixels which make the display and put a camera in front. 
  The camera iteratively considers rays going through these pixels, and the ray marching algorithm is used to decide if the ray collides 
  with an object and if so what the depth is. 
  
  # Ray marching algorithm 
  The goal of ray marching is to decide if a ray collides with an object in the scene and if so what the depth is. 
  Each ray is described by a direction vector. Since the SDF returns the shortest distance to the surface of an object, we can safely move 
  that far along the direction vector. This happens iteratively until either the SDF returns (about) 0 (indicating that we are at the surface), 
  the number of iterations reaches a limit, or the total distance moved exceeds the draw distance. 
  
