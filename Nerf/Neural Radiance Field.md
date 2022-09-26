# OS1-128-


What is a NeRF (Neural Radiance Field)?
A NeRF (Neural Radiance Field) is neural network that non-discretely models a point and viewing direction in 3D space to the amount of light that is being emitted by that point in each direction. It allows us to synthesize novel (new) views of complex scenes.
To clarify, let's break it down into its component parts: 

   - The word neural obviously means that there's a Neural Network involved
   - Radiance refers to the radiance of the scene that the Neural Network outputs. 
    It is basically describing how much light is being emitted by a point in space in each direction.
   - The word Field means that the Neural Network models a continuous and non-discretized representation of the scene that it learns.



The Neural Network F in this case is a simple Multi-layered Perceptron or MLP with ReLU activation.
The MLP consists of 9 fully-connected layers of width 256.

The Input to the MLP consists of 2 components:
- (x,y,z): which denotes the spatial position of a given point in 3D space.
- (θ,ϕ)(\theta, \phi):  denotes a given viewing direction from the point.

The output to the MLP consists of 2 components as well:
(r,g,b): which denotes the view composed from the point (x,y,z)  along the direction (θ,ϕ)(\theta, \phi) in RGB colorspace.

(σ\sigma): denotes the density or transparency of the point.

 ..





