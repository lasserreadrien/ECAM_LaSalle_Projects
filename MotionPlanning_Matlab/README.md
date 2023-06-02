# MotionPlanning tutorial using Octave (MATLAB) software

Some functions were already given by our teacher and some were created to achieve the goal of the project: building motion plans with RRT (rapidly exploring random tree algorithm) and PRM (probabilistic roadmap).
To run the results use the following command under Octave software:

pkg load matgeom
buildPRM([-180 180; -180 180],10,2,1, 'mapTest')
buildRRT([-180 180; -180 180],45,2,1, 0.75, [2 0], [-2, 0])
