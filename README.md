# Assignment:

following (maybe) the trace of the class for vectors, make

(1) A class for Quaternions
(2) A class for Transforms

## The Quaternions Class

Suggestions: two fields, for the imaginary (a 3D vector) and a real parts.

Methods / global functions to support: 

General methods for quaternions:
- linear ops (scaling, summing) [operator overloading suggested]
- squared norm, norm and normalization
- multiplication
- conjugation (two methods: conjugate and conjugated)
- dot products (as 4D vectors)
- lerp (linear, no checks)
- areEqual (given two quaternions, are they the same? -- up to tolerance)

Methods specific for unitary quats (representing rotations)
- mix (with shortest path, renormalization)
- construction from axis and angle
- construction from euler angles 
- export as 3x3 matrix
- apply to point/vector/versor
- again, but optimized (see slides)
- inversion (of rotation)
- areEquivalent (given two quaternions, do they represent the same rot? -- up to tolerance)

OPTIONAL methods/functions (more difficult):
- construction from 3x3 matrix 
- export as axis and angle
- export as euler angles

[also make a quick class for 3x3 matrix. Three fields: the column vectors. Just one method: mult a 3D vector]

TESTING:
- construct a quaternion that rotatates by 180 deg around Y:
  check if it behaves as expected by rotating a point
  check that its inverse is itself (is equivalent to itself)
- construct a rot around an arbitrary oblique axis, by 60 deg.
  Apply it 6 times to an arbitrary point. It should go back to itself!
- turn arbitrary quat into matrix. Check that rotating an arbitrary point is the same as multiplying the matrix

## The transform class (optional)

Follow the slides for methods 
(applyToPoints, applyToVersor, applyToVectors, invert, cumultate, mix)
and fields (scale, rot, translate)

Produce an affine transform! (as a 3x3 matrix plus a vector)

Produce a affine 4x4 matrix (as a pointer to 16 floats, ready to be fed to OpenGL or DirectX)

TESTING: (more optional)
- make a mesh class (vector of positions, vector of faces)
- add a import and an export method
  in any way (an easy way: turn the mesh in OFF format first using MESHLAB then read it using std::fstream c++ -- 
  export mesh using MESHLAB)
- load a mehs, construct an arbitrary transform, apply the transform to all vertices, export the mesh
- inspect the results using (e.g.) meshlab



