# Mini-repport for the Hough transform part:
## Why is the accumulator setup with 3 dimensions, what are the range and discretisation required for each dimension?
The accumulator needs to be on 3 dimensions because we are estimating 3 parameters. On the equation z = a*x + b*y + c we want to estimate parameters a, b and c. Therefore each dimesion corresponds to the possible values for each parameter.   
Given that we are going from a continuous world to a dicrete array, we need to discretize the real values. Moreover, our array has finite bounds unlike the real world. This is why we need to specify the bound in which we are looking for the best parameters.

## Discretisation implies aliasing. Propose (but do not necessarily implement) a way to combing the Hough Transform with other tools to have a precise estimator while still being resilient to noise and outliers.
The Hough transfors is already very robust against noise and outliers. However, to get a better result we could run two Hough tranfors. The first one to get an idea of the scale of the parameters and then a second shorter one to get a better precision.

## Evaluation
The hough transoform is way more robust than other parameters we have seen so far. However, it is very slow. This is why the transisions are not smooth and we have to allow some time before getting an optimal result. Not all the parameters have the same impact. For instance, the constant we add to the plane needs to be more precise than the other two. It is more visible when the plane is offset from the real ground.