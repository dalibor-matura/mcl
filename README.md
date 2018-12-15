# MCL (Modern Collision Library)

The branch of FCL - Fast Collision Library (Flexible Collision Library).

In nutshell, I used information from "Continuous Collision Detection for Articulated Models using Taylor Models and Temporal Culling" paper http://graphics.ewha.ac.kr/CATCH/ and implemented:
* Articulated model for robotic arms
* Third order smooth interpolation
* And motion bound calculation

That all together allowed Continuous Collision detection via Conservative Advancement steps (defined by Brian Mirtich's) on multi-axis robotic arms and devices with arbitrary number of articulated joints. Motion interpolations can be adjusted to many motion types that can be simulated via third order smooth interpolations.

Acording to test done at the beginning of 2014 by me and my friends, it was the Fastest Continous Collision detection library that can satisfy multi-axis robotic arms and devices with arbitrary number of articulated joints, like microscope chamber arm.

I had an idae how it could be improved and what a potential product it might evolve to:
- MCL brochure https://github.com/dalibor-matura/mcl/raw/master/MCL-brochure.pdf
- MCL presentation on youtube: https://youtu.be/o3n1Gy2HENI
