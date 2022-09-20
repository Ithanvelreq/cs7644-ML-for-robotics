# Mini-repport for the Hough transform part:
## Find good parameters in terms of number of samples and tolerance. Try to find physical/mathematical justification for your choice of parameters.  
We tried to pick a very small tolerance and a very high number of iterations in order to find the plane. The issue with this Ransac is that it will not find the same each time we call the function on the same set of points. The numbers that worked nicely for us were 1e-7 and 1000 iterations.

## Evaluate the sensitivity of the plane estimation to environment parameters, in particular smooth slope transition and obstacles.  
Once again, the model does not find the same plane for the same set of points time it is executed. So it is not smooth at all. However, when we are at a perfecly flat ground, we get a nice plane. It computes it faster than the other approaches we have tried so far.

## Propose a solution to use RANSAC as a first step into a precise least-square-based ground planedetection.  
We could do a linear regression on the points that are considered inliers by the ransac algorithm. Ransac does the preprocessing needed to get a good dataset before performin linear regression which is more consistent than Ransac.